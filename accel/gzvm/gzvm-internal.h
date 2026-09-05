#ifndef GZVM_INTERNAL_H
#define GZVM_INTERNAL_H

#include "qemu/osdep.h"
#include <sys/ioctl.h>
#include "qemu/typedefs.h"
#include "hw/core/cpu.h"
#include "system/gzvm.h"
#include "system/gzvm_int.h"
#include "linux-headers/linux/gzvm.h"

#define gzvm_slots_lock(s)    qemu_mutex_lock(&(s)->slots_lock)
#define gzvm_slots_unlock(s)  qemu_mutex_unlock(&(s)->slots_lock)
gzvm_slot *gzvm_find_slot_by_addr_locked(GZVMState *s, uint64_t addr);
void gzvm_install_sigsegv_handler(void);
void gzvm_init_vcpu_sigsegv(void);
void gzvm_signal_update_regions(GZVMState *s);
void gzvm_cleanup_mem_state(void);
int gzvm_dev_ioctl(GZVMState *s, int type, void *arg);
void gzvm_ioctl_set_state(GZVMState *s);

void gzvm_cpu_kick_self(void);
void gzvm_init_cpu_signals(void);
void gzvm_eat_signals(CPUState *cpu);

int gzvm_handle_mmio_exit(CPUState *cpu, struct gzvm_vcpu_run *run);
int gzvm_handle_system_event(CPUState *cpu, struct gzvm_vcpu_run *run);
int gzvm_handle_fail_entry(CPUState *cpu, struct gzvm_vcpu_run *run);
int gzvm_handle_internal_error(CPUState *cpu, struct gzvm_vcpu_run *run);
int gzvm_handle_unknown_exit(CPUState *cpu, struct gzvm_vcpu_run *run);

/* gzvm_add_irqfd() / gzvm_remove_irqfd() are declared in system/gzvm.h. */
extern MemoryListener gzvm_ioeventfd_listener;
extern MemoryListener gzvm_io_listener;

#endif
