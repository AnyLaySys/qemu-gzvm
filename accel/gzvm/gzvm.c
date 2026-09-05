#include "qemu/osdep.h"
#include <sys/ioctl.h>
#include "qemu/error-report.h"
#include "qemu/atomic.h"
#include "hw/core/cpu.h"
#include "system/cpus.h"
#include "system/gzvm.h"
#include "system/gzvm_int.h"
#include "linux-headers/linux/gzvm.h"
#include "exec/cpu-common.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "qemu/main-loop.h"
#include "system/runstate.h"
#include "qemu/guest-random.h"
#include "gzvm-internal.h"

void gzvm_embedded_cleanup(void)
{
    GZVMState *s;
    CPUState *cpu;
    bool unlock;

    if (!gzvm_allowed || !current_accel())
        return;
    s = GZVM_STATE(current_accel());
    qatomic_set(&gzvm_vm_stopped, true);
    unlock = !bql_locked();
    if (unlock) bql_lock();
    CPU_FOREACH(cpu) {
        if (cpu->created)
            cpu_remove_sync(cpu);
    }
    if (s->slots)
        gzvm_cleanup_mem_state();
    if (s->vmfd >= 0) close(s->vmfd);
    if (s->fd >= 0) close(s->fd);
    if (unlock) bql_unlock();
}

static int gzvm_init_vcpu(CPUState *cpu)
{
    struct GZVCPUState *vcpu = g_new0(struct GZVCPUState, 1);
    int ret;

    ret = gzvm_vm_ioctl(GZVM_CREATE_VCPU, (void *)(uintptr_t)cpu->cpu_index);
    if (ret < 0) {
        g_free(vcpu);
        gz_report("gzvm: GZVM_CREATE_VCPU failed: %s (errno=%d)",
                     strerror(errno), errno);
        return ret;
    }

    vcpu->fd = ret;
    vcpu->run = g_new0(struct gzvm_vcpu_run, 1);
    qatomic_set(&cpu->accel, (AccelCPUState *)vcpu);
    return 0;
}

static int gzvm_cpu_exec(CPUState *cpu)
{
    struct gzvm_vcpu_run *run = GZVCPU(cpu)->run;
    int ret;

    if (qatomic_read(&gzvm_vm_stopped)) {
        qatomic_set(&cpu->halted, 1);
        return EXCP_INTERRUPT;
    }

    run->immediate_exit = 0;
    bql_unlock();
    if (qatomic_load_acquire(&cpu->exit_request)) {
        gzvm_cpu_kick_self();
    }
    ret = gzvm_vcpu_ioctl(cpu, GZVM_RUN, run);
    bql_lock();

    /*
     * Clear the kick.  cpu_exit() in hw/core/cpu-common.c sets exit_request and
     * nothing in the tree ever clears it, so without this the first kick a vCPU
     * receives latches for the life of the VM: every later entry here calls
     * gzvm_cpu_kick_self(), which sets run->immediate_exit, and the host driver
     * honours that at the top of GZVM_RUN ("if (vcpu->run->immediate_exit == 1)"
     * in gzvm_vcpu.c) and returns without entering the guest.
     *
     * That vCPU then makes no forward progress at all, and it does so at full
     * speed: gzvm_eat_signals() clears immediate_exit, we return EXCP_INTERRUPT,
     * gzvm_vcpu_thread_is_idle() returns false so qemu_wait_io_event() does not
     * park, and we come straight back here to set immediate_exit again.  Each
     * turn of that loop takes and drops the BQL, and the BQL is global, so a
     * single stuck vCPU starves every other vCPU, the main loop and the
     * iothreads.  Nothing deadlocks, so the guest reports no hung tasks and no
     * RCU stalls -- it just runs orders of magnitude too slowly, and the more
     * vCPUs there are the likelier it is that at least one has been kicked.
     *
     * Upstream kvm_cpu_exec() clears exit_request at the end of every call for
     * exactly this reason.  Done here rather than after the exit_reason switch
     * below because that switch returns from many places.
     */
    qatomic_set(&cpu->exit_request, 0);

    if (ret < 0) {
        if (errno == EINTR || errno == EAGAIN) {
            gzvm_eat_signals(cpu);
            return EXCP_INTERRUPT;
        }
        gz_report("gzvm: GZVM_RUN failed: %s (errno=%d)", strerror(errno), errno);
        return -1;
    }

    switch (run->exit_reason) {
    case GZVM_EXIT_MMIO: {
        return gzvm_handle_mmio_exit(cpu, run);
    }
    case GZVM_EXIT_SYSTEM_EVENT:
        return gzvm_handle_system_event(cpu, run);
    case GZVM_EXIT_FAIL_ENTRY:
        return gzvm_handle_fail_entry(cpu, run);
    case GZVM_EXIT_INTERNAL_ERROR:
        return gzvm_handle_internal_error(cpu, run);
    case GZVM_EXIT_IDLE:
        return EXCP_INTERRUPT;
    case GZVM_EXIT_IRQ:
        return EXCP_INTERRUPT;
    case GZVM_EXIT_HYPERCALL:
        gz_report("gzvm: VCPU%u unhandled hypercall fn=0x%" PRIx64,
                    cpu->cpu_index, (uint64_t)run->hypercall.args[0]);
        return EXCP_INTERRUPT;
    case GZVM_EXIT_GZ:
        return EXCP_INTERRUPT;
    case GZVM_EXIT_IPI:
        return EXCP_INTERRUPT;
    case GZVM_EXIT_DEBUG:
        return EXCP_DEBUG;
    case GZVM_EXIT_SHUTDOWN:
        qatomic_set(&gzvm_vm_stopped, true);
        qatomic_set(&cpu->halted, 1);
        if (cpu->cpu_index == 0) {
            qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
            return EXCP_INTERRUPT;
        }
        return EXCP_INTERRUPT;
    case GZVM_EXIT_EXCEPTION:
        gz_report("gzvm: VCPU%u exception: type=%u error_code=0x%x "
                     "fault_gpa=0x%" PRIx64,
                     cpu->cpu_index, run->exception.exception,
                     run->exception.error_code,
                     (uint64_t)run->exception.fault_gpa);
        return -1;
    case 0:
        gz_report_once("gzvm: VCPU%u exit_reason=0 (vCPU may not have run)",
                         cpu->cpu_index);
        return 0;
    default:
        return gzvm_handle_unknown_exit(cpu, run);
    }
}

static void do_gzvm_cpu_synchronize_post_reset(CPUState *cpu,
                                               run_on_cpu_data arg)
{
    int ret = gzvm_arch_put_registers(cpu, 0);
    if (ret) {
        gz_report("gzvm: VCPU%u put_registers failed with %d",
                    cpu->cpu_index, ret);
    }
}

void gzvm_cpu_synchronize_post_reset(CPUState *cpu)
{
    run_on_cpu(cpu, do_gzvm_cpu_synchronize_post_reset, RUN_ON_CPU_NULL);
}

static void do_gzvm_cpu_synchronize_post_init(CPUState *cpu,
                                              run_on_cpu_data arg)
{
    int ret = gzvm_arch_put_registers(cpu, 1);
    if (ret) {
        gz_report("gzvm: VCPU%u put_registers(post_init) failed with %d",
                    cpu->cpu_index, ret);
    }
}

void gzvm_cpu_synchronize_post_init(CPUState *cpu)
{
    run_on_cpu(cpu, do_gzvm_cpu_synchronize_post_init, RUN_ON_CPU_NULL);
}

static bool gzvm_cpu_thread_init(CPUState *cpu)
{
    rcu_register_thread();

    bql_lock();
    qemu_thread_get_self(cpu->thread);
    cpu->thread_id = qemu_get_thread_id();
    current_cpu = cpu;

    gzvm_init_cpu_signals();
    gzvm_init_vcpu_sigsegv();

    if (gzvm_init_vcpu(cpu)) {
        cpu_thread_signal_destroyed(cpu);
        bql_unlock();
        rcu_unregister_thread();
        return false;
    }

    cpu_thread_signal_created(cpu);
    qemu_guest_random_seed_thread_part2(cpu->random_seed);
    return true;
}

static void gzvm_cpu_thread_cleanup(CPUState *cpu)
{
    struct GZVCPUState *vcpu = GZVCPU(cpu);

    close(vcpu->fd);
    qatomic_set(&cpu->accel, NULL);
    g_free(vcpu->run);
    g_free(vcpu);
    cpu_thread_signal_destroyed(cpu);
    bql_unlock();
    rcu_unregister_thread();
}

void *gzvm_cpu_thread_fn(void *arg)
{
    CPUState *cpu = arg;
    int ret;

    if (!gzvm_cpu_thread_init(cpu)) {
        return NULL;
    }

    do {
        if (cpu_can_run(cpu)) {
            ret = gzvm_cpu_exec(cpu);
            if (ret == EXCP_DEBUG) {
                cpu_handle_guest_debug(cpu);
            } else if (ret < 0) {
                gz_report("gzvm: VCPU%u run error ret=%d", cpu->cpu_index, ret);
                vm_stop(RUN_STATE_INTERNAL_ERROR);
            }
        }
        /*
         * qemu_wait_io_event(), not qemu_wait_io_event_common(): the latter
         * never blocks, so while the VM is not yet running (cpu_can_run() is
         * false during machine init, when runstate is still PRELAUNCH) this
         * loop would spin without ever reaching gzvm_cpu_exec() -- the only
         * place that releases the BQL.  The vCPU thread then holds the BQL
         * forever and the main thread can never finish machine init or reach
         * vm_start(), so the VM hangs before any firmware or kernel runs.
         *
         * qemu_wait_io_event() parks on cpu->halt_cond while
         * cpu_thread_is_idle() holds, which releases the BQL; resume_all_vcpus()
         * clears cpu->stopped and broadcasts halt_cond to wake us.
         */
        qemu_wait_io_event(cpu);
    } while (!cpu->unplug || cpu_can_run(cpu));

    gzvm_cpu_thread_cleanup(cpu);
    return NULL;
}
