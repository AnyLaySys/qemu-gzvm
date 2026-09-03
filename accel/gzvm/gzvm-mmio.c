#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "exec/cpu-common.h"
#include "hw/core/cpu.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "system/runstate.h"
#include "system/gzvm.h"
#include "system/gzvm_int.h"
#include "linux-headers/linux/gzvm.h"
#include "gzvm-internal.h"

int gzvm_handle_mmio_exit(CPUState *cpu, struct gzvm_vcpu_run *run)
{
    hwaddr addr = run->mmio.phys_addr;
    AccelState *accel = current_accel();
    GZVMState *s = accel ? GZVM_STATE(accel) : NULL;
    uint8_t *slot_mem = NULL;
    uint64_t slot_start = 0, slot_size = 0;
    MemTxAttrs attrs = { .secure = true };
    MemTxResult r;

    if (run->mmio.size > 8) {
        gz_report("gzvm: large MMIO %s at 0x%" PRIx64 " size=%" PRIu64
                     " (max 8 bytes supported, treated as RAZ/WI)",
                     run->mmio.is_write ? "write" : "read",
                     (uint64_t)run->mmio.phys_addr,
                     (uint64_t)run->mmio.size);
        return 0;
    }

    if (s) {
        gzvm_slots_lock(s);
        gzvm_slot *slot = gzvm_find_slot_by_addr_locked(s, addr);
        if (slot && slot->mem) {
            attrs = MEMTXATTRS_UNSPECIFIED;
            slot_mem = slot->mem;
            slot_start = slot->start;
            slot_size = slot->size;
        }
        gzvm_slots_unlock(s);
    }

    r = address_space_rw(&address_space_memory, addr, attrs,
                         run->mmio.data, run->mmio.size, run->mmio.is_write);
    if (r == MEMTX_OK) {
        return 0;
    }

    if (slot_mem) {
        uint64_t offset = addr - slot_start;
        if (offset < slot_size) {
            size_t xlen = MIN((uint64_t)run->mmio.size, slot_size - offset);
            if (run->mmio.is_write) {
                memcpy(slot_mem + offset, run->mmio.data, xlen);
            } else {
                memcpy(run->mmio.data, slot_mem + offset, xlen);
            }
            return 0;
        }
    }

    gz_report("gzvm: %s at 0x%" PRIx64 " size=%" PRIu64 " returned %u, "
                "treated as RAZ/WI",
                run->mmio.is_write ? "MMIO write" : "MMIO read",
                (uint64_t)run->mmio.phys_addr,
                (uint64_t)run->mmio.size, r);
    return 0;
}

int gzvm_handle_system_event(CPUState *cpu, struct gzvm_vcpu_run *run)
{
    switch (run->system_event.type) {
    case GZVM_SYSTEM_EVENT_SHUTDOWN:
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
        return EXCP_INTERRUPT;
    case GZVM_SYSTEM_EVENT_RESET:
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        return EXCP_INTERRUPT;
    case GZVM_SYSTEM_EVENT_CRASH:
        qemu_system_guest_panicked(cpu_get_crash_info(cpu));
        return 0;
    case GZVM_SYSTEM_EVENT_WAKEUP:
        cpu->halted = 0;
        return EXCP_INTERRUPT;
    case GZVM_SYSTEM_EVENT_SUSPEND:
    case GZVM_SYSTEM_EVENT_S2IDLE:
        cpu->halted = 1;
        return EXCP_INTERRUPT;
    case GZVM_SYSTEM_EVENT_SEV_TERM:
        gz_report("gzvm: SEV_TERM event on VCPU%u (not applicable to GZVM)",
                    cpu->cpu_index);
        return EXCP_INTERRUPT;
    default:
        return 0;
    }
}

int gzvm_handle_fail_entry(CPUState *cpu, struct gzvm_vcpu_run *run)
{
    gz_report("gzvm: CPU#%d FAIL_ENTRY reason=0x%" PRIx64 " cpu=%u",
                 cpu->cpu_index,
                 (uint64_t)run->fail_entry.hardware_entry_failure_reason,
                 run->fail_entry.cpu);
    return -1;
}

int gzvm_handle_internal_error(CPUState *cpu, struct gzvm_vcpu_run *run)
{
    gz_report("gzvm: CPU#%d INTERNAL_ERROR suberror=%u ndata=%u",
                 cpu->cpu_index, run->internal.suberror, run->internal.ndata);
    for (int i = 0; i < run->internal.ndata && i < 16; i++) {
        gz_report("gzvm:   data[%d] = 0x%" PRIx64, i,
                     (uint64_t)run->internal.data[i]);
    }
    return -1;
}

int gzvm_handle_unknown_exit(CPUState *cpu, struct gzvm_vcpu_run *run)
{
    gz_report("gzvm: CPU#%d unknown exit_reason=0x%x",
                 cpu->cpu_index, run->exit_reason);
    return 0;
}
