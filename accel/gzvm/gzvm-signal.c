/*
 * Demand-paging fallback for GZVM guest memory.
 *
 * QEMU maps guest RAM with mmap() but the kernel does not back every page with
 * physical memory upfront.  When the GZ hypervisor accesses an unmapped page
 * it cannot satisfy the fault itself and the host process receives SIGBUS.
 *
 * The handler below catches that SIGBUS, checks whether the faulting address
 * falls inside a registered gzvm memory slot, and if so mmap()s a fresh
 * zero page at that address.  The kernel driver's demand-paging path
 * (gzvm_handle_page_fault() -> pin_user_pages()) can then pin the page and
 * map it into the guest's physical address space.
 *
 * Without this layer the kernel driver's pin_user_pages() would fail with
 * -EFAULT because the host VMA has no backing page yet.
 */
#include "qemu/osdep.h"
#include <sys/mman.h>
#include "qemu/error-report.h"
#include "system/gzvm.h"
#include "system/gzvm_int.h"
#include "gzvm-internal.h"

static uintptr_t gzvm_signal_page_size;

#define GZVM_SIGNAL_MAX_REGIONS 64
typedef struct {
    uintptr_t start;
    uintptr_t end;
} GZVMSignalHvaRange;

static GZVMSignalHvaRange gzvm_signal_hva_ranges[GZVM_SIGNAL_MAX_REGIONS];
static int gzvm_signal_nr_hva_ranges;

/*
 * Snapshot the HVA ranges of all active gzvm memory slots so the signal
 * handler can quickly decide whether a faulting address is ours.
 * Called from gzvm_region_commit() whenever slots change.
 */
void gzvm_signal_update_regions(GZVMState *s)
{
    gzvm_signal_nr_hva_ranges = 0;
    for (int i = 0; i < (int)s->nr_active_slots &&
                gzvm_signal_nr_hva_ranges < GZVM_SIGNAL_MAX_REGIONS; i++) {
        gzvm_slot *slot = &s->slots[s->sorted_ids[i]];
        if (slot->mem) {
            int idx = gzvm_signal_nr_hva_ranges++;
            gzvm_signal_hva_ranges[idx].start = (uintptr_t)slot->mem;
            gzvm_signal_hva_ranges[idx].end = (uintptr_t)slot->mem +
                                              slot->size;
        }
    }
}

static void gzvm_signal_diagnostic(int sig, siginfo_t *si, void *ctx)
{
    ucontext_t *uc = ctx;
    char buf[256];
    int len;

    len = snprintf(buf, sizeof(buf),
                   "Signal: %d (%s)\n"
                   "Faulting address: %p\n"
                   "si_code: %d\n",
                   sig, sig == SIGSEGV ? "SIGSEGV" :
                   sig == SIGBUS ? "SIGBUS" : "?", si->si_addr,
                   si->si_code);
    if (len > 0) {
        write(STDERR_FILENO, buf, len);
    }
    if (uc) {
        len = snprintf(buf, sizeof(buf), "PC: 0x%llx\nLR: 0x%llx\n",
                       (unsigned long long)uc->uc_mcontext.pc,
                       (unsigned long long)uc->uc_mcontext.regs[30]);
        if (len > 0) {
            write(STDERR_FILENO, buf, len);
        }
    }
}

/*
 * SIGBUS handler for demand paging.  Only SIGBUS is meaningful here:
 * SIGSEGV is caught but forwarded to the default handler unless it
 * also happens to be a bus-error variant (some kernels deliver page-fault
 * failures as SIGBUS, others as SIGSEGV -- we handle both for safety).
 *
 * If the fault address is inside a gzvm memory slot we mmap() a zero page
 * so the kernel driver's pin_user_pages() can succeed on retry.
 * MAP_FIXED_NOREPLACE is preferred to avoid silently clobbering an existing
 * mapping (available since Linux 4.17); older kernels fall back to MAP_FIXED.
 * If mmap() fails (e.g. page already mapped by another thread racing us) we
 * fall through to the default handler.
 */
static void gzvm_sigsegv_handler(int sig, siginfo_t *si, void *ctx)
{
    if (sig == SIGBUS && si->si_addr) {
        uintptr_t page_mask = ~(gzvm_signal_page_size - 1);
        uintptr_t page_addr = (uintptr_t)si->si_addr & page_mask;
        int map_flags = MAP_PRIVATE | MAP_ANONYMOUS;
        void *ret;
        bool in_gzvm = false;

        /*
         * Walk the snapshot of gzvm memory slot HVAs to decide whether
         * this fault is ours.  A linear scan is fine -- there are at most
         * GZVM_SIGNAL_MAX_REGIONS (64) entries and this path is cold.
         */
        for (int i = 0; i < gzvm_signal_nr_hva_ranges; i++) {
            if ((uintptr_t)si->si_addr >= gzvm_signal_hva_ranges[i].start &&
                (uintptr_t)si->si_addr < gzvm_signal_hva_ranges[i].end) {
                in_gzvm = true;
                break;
            }
        }

        if (in_gzvm) {
            /*
             * Map a fresh zero page at the faulting address.  This gives
             * the host process valid backing memory so the kernel driver's
             * pin_user_pages() can succeed when the GZ hypervisor retries.
             */
#ifdef MAP_FIXED_NOREPLACE
            map_flags |= MAP_FIXED_NOREPLACE;
#else
            map_flags |= MAP_FIXED;
#endif
            ret = mmap((void *)page_addr, gzvm_signal_page_size,
                       PROT_READ | PROT_WRITE, map_flags, -1, 0);
            if (ret != MAP_FAILED) {
                return;
            }
#ifdef MAP_FIXED_NOREPLACE
            /*
             * Another vCPU thread raced us and already mapped this page.
             * That is fine -- the page exists now, so pin_user_pages() will
             * succeed.  Fall through to restore the default handler.
             */
            if (errno == EEXIST) {
                return;
            }
#endif
        }
    }

    /*
     * Not a gzvm fault or mmap() failed -- restore the default handler
     * and re-raise so the process gets the normal crash behaviour.
     */
    gzvm_signal_diagnostic(sig, si, ctx);
    {
        struct sigaction dfl = { .sa_handler = SIG_DFL };
        sigaction(sig, &dfl, NULL);
    }
    raise(sig);
}

/*
 * Block SIGBUS/SIGSEGV in the main thread during early init so they
 * cannot be delivered before the handler is installed.  The signals are
 * unblocked once gzvm_init_vcpu_sigsegv() installs the handler in each
 * vCPU thread.
 */
void gzvm_install_sigsegv_handler(void)
{
    sigset_t set;

    gzvm_signal_page_size = qemu_real_host_page_size();

#ifndef MAP_FIXED_NOREPLACE
    gz_report("gzvm: MAP_FIXED_NOREPLACE not available (kernel < 4.17), "
                "falling back to MAP_FIXED");
#endif

    sigemptyset(&set);
    sigaddset(&set, SIGBUS);
    sigaddset(&set, SIGSEGV);
    pthread_sigmask(SIG_BLOCK, &set, NULL);
}

/*
 * Install the demand-paging signal handler in the calling (vCPU) thread
 * and unblock SIGBUS/SIGSEGV so the handler can fire.  Each vCPU thread
 * calls this during init because signal masks are per-thread.
 */
void gzvm_init_vcpu_sigsegv(void)
{
    struct sigaction sa;
    sigset_t set;

    memset(&sa, 0, sizeof(sa));
    sa.sa_sigaction = gzvm_sigsegv_handler;
    sa.sa_flags = SA_SIGINFO | SA_RESTART;
    sigfillset(&sa.sa_mask);
    sigaction(SIGSEGV, &sa, NULL);
    sigaction(SIGBUS, &sa, NULL);

    sigemptyset(&set);
    sigaddset(&set, SIGBUS);
    sigaddset(&set, SIGSEGV);
    pthread_sigmask(SIG_UNBLOCK, &set, NULL);
}
