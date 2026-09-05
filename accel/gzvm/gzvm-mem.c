#include "qemu/osdep.h"
#include <sys/ioctl.h>
#include "qemu/error-report.h"
#include "exec/cpu-common.h"
#include "hw/boards.h"
#include "hw/core/cpu.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "system/gzvm.h"
#include "system/gzvm_int.h"
#include "linux-headers/linux/gzvm.h"
#include "gzvm-internal.h"

static void gzvm_assert_mutex_locked(QemuMutex *m)
{
    int ret = pthread_mutex_trylock(&m->lock);
    if (ret == 0) {
        pthread_mutex_unlock(&m->lock);
    }
    assert(ret == EBUSY);
}

static int gzvm_find_first_ge(GZVMState *s, uint64_t addr)
{
    int lo = 0, hi = (int)s->nr_active_slots - 1;
    int first_ge = (int)s->nr_active_slots;
    while (lo <= hi) {
        int mid = lo + (hi - lo) / 2;
        gzvm_slot *slot = &s->slots[s->sorted_ids[mid]];
        if (slot->start >= addr) {
            first_ge = mid;
            hi = mid - 1;
        } else {
            lo = mid + 1;
        }
    }
    return first_ge;
}

static gzvm_slot *gzvm_find_matching_slot_locked(GZVMState *s,
                                                  uint64_t start,
                                                  uint64_t size, void *mem)
{
    gzvm_assert_mutex_locked(&s->slots_lock);

    for (guint i = 0; i < s->nr_active_slots; i++) {
        gzvm_slot *slot = &s->slots[s->sorted_ids[i]];

        if (slot->start == start && slot->size == size && slot->mem == mem) {
            return slot;
        }
    }
    return NULL;
}

gzvm_slot *gzvm_find_slot_by_addr_locked(GZVMState *s, uint64_t addr)
{
    gzvm_assert_mutex_locked(&s->slots_lock);
    if (!s->nr_active_slots) return NULL;
    int pos = gzvm_find_first_ge(s, addr + 1);
    if (pos > 0) {
        gzvm_slot *slot = &s->slots[s->sorted_ids[pos - 1]];
        if (addr - slot->start < slot->size) return slot;
    }
    return NULL;
}

gzvm_slot *gzvm_find_slot_by_addr(uint64_t addr)
{
    AccelState *accel = current_accel();
    GZVMState *s;

    if (!accel) {
        return NULL;
    }
    s = GZVM_STATE(accel);

    gzvm_slots_lock(s);
    gzvm_slot *slot = gzvm_find_slot_by_addr_locked(s, addr);
    gzvm_slots_unlock(s);
    return slot;
}

static gzvm_slot *gzvm_get_free_slot(GZVMState *s)
{
    for (guint i = 0; i < s->nr_slots_allocated; i++)
        if (!s->slots[i].size)
            return &s->slots[i];
    uint32_t old = s->nr_slots_allocated;
    uint32_t nslots = MIN(old * 2, GZVM_MAX_MEM_SLOTS);
    if (nslots <= old) return NULL;
    s->slots = g_renew(gzvm_slot, s->slots, nslots);
    memset(s->slots + old, 0, (nslots - old) * sizeof(gzvm_slot));
    s->sorted_ids = g_renew(gint, s->sorted_ids, nslots);
    for (uint32_t i = old; i < nslots; i++) s->slots[i].id = i;
    s->nr_slots_allocated = nslots;
    return &s->slots[old];
}

static void gzvm_update_sorted_ids(GZVMState *s, int slot_id, bool add)
{
    int pos;
    if (add) {
        pos = gzvm_find_first_ge(s, s->slots[slot_id].start);
        memmove(&s->sorted_ids[pos + 1], &s->sorted_ids[pos],
                (s->nr_active_slots - (uint32_t)pos) * sizeof(gint));
        s->sorted_ids[pos] = slot_id;
        s->nr_active_slots++;
    } else {
        uint64_t gpa = s->slots[slot_id].start;
        pos = gzvm_find_first_ge(s, gpa);
        while (pos < (int)s->nr_active_slots &&
               s->slots[s->sorted_ids[pos]].start == gpa) {
            if (s->sorted_ids[pos] == slot_id) {
                memmove(&s->sorted_ids[pos], &s->sorted_ids[pos + 1],
                        (s->nr_active_slots - (uint32_t)pos - 1) * sizeof(gint));
                s->nr_active_slots--;
                return;
            }
            pos++;
        }
    }
}

static int
gzvm_set_memory_region_locked(GZVMState *s, uint32_t slot, uint32_t flags,
                              uint64_t gpa, uint64_t size, void *hva)
{
    struct gzvm_userspace_memory_region gumr = {
        .slot = slot,
        .flags = flags,
        .guest_phys_addr = gpa,
        .memory_size = size,
        .userspace_addr = (__u64)(uintptr_t)hva,
    };
    gzvm_assert_mutex_locked(&s->slots_lock);
    return gzvm_vm_ioctl(GZVM_SET_USER_MEMORY_REGION, &gumr);
}

static void gzvm_mem_slot_deactivate_locked(GZVMState *s, gzvm_slot *slot)
{
    gzvm_assert_mutex_locked(&s->slots_lock);
    gzvm_update_sorted_ids(s, slot->id, false);
    slot->size = 0;
    slot->mem = NULL;
    slot->start = 0;
    slot->flags = 0;
    gzvm_signal_update_regions(s);
}

static int gzvm_remove_mem_slot_locked(GZVMState *s, gzvm_slot *slot)
{
    int ret;

    gzvm_assert_mutex_locked(&s->slots_lock);

    ret = gzvm_set_memory_region_locked(s, slot->id, 0, 0, 0, NULL);
    if (ret) {
        gz_report("gzvm: remove memory slot %u failed: %s (errno=%d)",
                     slot->id, strerror(errno), errno);
        return ret;
    }

    gzvm_mem_slot_deactivate_locked(s, slot);
    return 0;
}


static int gzvm_add_mem_slot(GZVMState *s, uint8_t *hva, uint64_t gpa,
                              uint64_t size,
                  uint32_t flags)
{
    gzvm_slot *slot;
    int ret;

    slot = gzvm_get_free_slot(s);
    if (!slot) {
        gz_report("gzvm: No free memory slots available!");
        return -ENOSPC;
    }

    ret = gzvm_set_memory_region_locked(s, slot->id, flags,
                                        gpa, size, hva);
    if (ret) {
        gz_report("gzvm: GZVM_SET_USER_MEMORY_REGION failed: %s (errno=%d)",
                     strerror(errno), errno);
        return ret;
    }

    slot->size = size;
    slot->mem = hva;
    slot->start = gpa;
    slot->flags = flags;

    gzvm_update_sorted_ids(s, slot->id, true);
    gzvm_signal_update_regions(s);
    return 0;
}

static int gzvm_add_mem(GZVMState *s, MemoryRegionSection *section,
                        uint32_t flags)
{
    MemoryRegion *area = section->mr;
    uint64_t total_size = int128_get64(section->size);
    uint8_t *base_hva = memory_region_get_ram_ptr(area) +
                        section->offset_within_region;
    uint64_t base_gpa = section->offset_within_address_space;

    return gzvm_add_mem_slot(s, base_hva, base_gpa, total_size, flags);
}

static void gzvm_set_phys_mem_locked(GZVMState *s,
                                     MemoryRegionSection *section, bool add)
{
    MemoryRegion *area = section->mr;
    MachineState *ms = MACHINE(qdev_get_machine());
    uint32_t flags = GZVM_USER_MEM_REGION_GUEST_MEM;
    uint64_t page_size = qemu_real_host_page_size();
    uint64_t section_start = section->offset_within_address_space;
    uint64_t section_size = int128_get64(section->size);
    uint8_t *section_hva;
    gzvm_slot *slot;

    if (!memory_region_is_ram(area) && !memory_region_is_rom(area) &&
        !memory_region_is_romd(area))
        return;

    if (!section_size || section_start > UINT64_MAX - section_size ||
        !QEMU_IS_ALIGNED(section_size, page_size) ||
        !QEMU_IS_ALIGNED(section_start, page_size))
        return;

    if (memory_region_is_rom(area) || memory_region_is_romd(area)) {
        if (section_start) {
            return;
        }
    } else if (section_start < s->ram_base ||
               section_start + section_size > s->ram_base + ms->ram_size) {
        return;
    }

    section_hva = memory_region_get_ram_ptr(area) +
                  section->offset_within_region;

    if (!add) {
        slot = gzvm_find_matching_slot_locked(s, section_start, section_size,
                                               section_hva);
        if (slot) {
            gzvm_remove_mem_slot_locked(s, slot);
        }
        return;
    }

    slot = gzvm_find_matching_slot_locked(s, section_start, section_size,
                                           section_hva);
    if (slot) {
        return;
    }

    /*
     * The kernel driver forwards this flags word verbatim to the hypervisor as
     * the MEMREGION_PURPOSE hypercall argument (see
     * gzvm_arch_memregion_purpose() in arch/arm64/geniezone/vm.c), so it is not
     * an advisory hint -- it tells GZ what the region *is*.  A read-only or
     * ROMD region is the firmware image (-bios installs virt.gzvm-firmware via
     * memory_region_init_rom_nomigrate(), which sets mr->readonly), and GZ
     * expects it to be declared as PROTECT_FW rather than ordinary guest RAM.
     * Declaring it as GUEST_MEM leaves the hypervisor's idea of the region's
     * purpose wrong for the whole life of the VM.
     *
     * A direct -kernel boot has no read-only region at all, which is why only
     * the UEFI path is affected.
     */
    if (area->readonly || area->rom_device) {
        flags = GZVM_USER_MEM_REGION_PROTECT_FW;
    }

    gzvm_add_mem(s, section, flags);
}

static void gzvm_region_add(MemoryListener *listener, MemoryRegionSection *section)
{
    GZVMState *s = GZVM_STATE(current_accel());
    GZVMMemoryUpdate *u;
    if (!s) return;
    u = g_new0(GZVMMemoryUpdate, 1);
    u->section = *section;
    QSIMPLEQ_INSERT_TAIL(&s->transaction_add, u, next);
}

static void gzvm_region_del(MemoryListener *listener, MemoryRegionSection *section)
{
    GZVMState *s = GZVM_STATE(current_accel());
    GZVMMemoryUpdate *u;
    if (!s) return;
    u = g_new0(GZVMMemoryUpdate, 1);
    u->section = *section;
    QSIMPLEQ_INSERT_TAIL(&s->transaction_del, u, next);
}

static void gzvm_drain_updates(GZVMState *s, bool add)
{
    QSIMPLEQ_HEAD(, GZVMMemoryUpdate) *q = add
        ? (void *)&s->transaction_add
        : (void *)&s->transaction_del;
    GZVMMemoryUpdate *u;
    while ((u = QSIMPLEQ_FIRST(q))) {
        QSIMPLEQ_REMOVE_HEAD(q, next);
        gzvm_set_phys_mem_locked(s, &u->section, add);
        g_free(u);
    }
}

static void gzvm_region_commit(MemoryListener *listener)
{
    GZVMState *s = GZVM_STATE(current_accel());
    if (!s) return;
    if (QSIMPLEQ_EMPTY(&s->transaction_add) &&
        QSIMPLEQ_EMPTY(&s->transaction_del))
        return;
    gzvm_slots_lock(s);
    gzvm_drain_updates(s, false);
    gzvm_drain_updates(s, true);
    gzvm_slots_unlock(s);
}

static MemoryListener gzvm_memory_listener = {
    .name = "gzvm",
    .priority = MEMORY_LISTENER_PRIORITY_ACCEL,
    .region_add = gzvm_region_add,
    .region_del = gzvm_region_del,
    .commit = gzvm_region_commit,
};

void gzvm_cleanup_mem_state(void)
{
    memory_listener_unregister(&gzvm_io_listener);
    memory_listener_unregister(&gzvm_ioeventfd_listener);
    memory_listener_unregister(&gzvm_memory_listener);
}

static int gzvm_create_vgic_device(GZVMState *s,
                                    int dev_type, uint64_t dev_addr,
                                    uint64_t dev_reg_size,
                                    uint64_t *base_out, const char *name)
{
    struct gzvm_create_device dev = {
        .dev_type = dev_type,
        .dev_addr = dev_addr,
        .dev_reg_size = dev_reg_size,
    };
    int ret = gzvm_vm_ioctl(GZVM_CREATE_DEVICE, &dev);
    if (ret) {
        gz_report("gzvm: create %s failed: %s (errno=%d)",
                     name, strerror(errno), errno);
        return ret;
    }
    *base_out = dev_addr;
    return 0;
}

static int gzvm_open_device(GZVMState *s)
{
    int ret;

    s->fd = qemu_open_old("/dev/gzvm", O_RDWR);
    if (s->fd == -1) {
        gz_report("Could not access /dev/gzvm: %s", strerror(errno));
        return -1;
    }

    ret = gzvm_dev_ioctl(s, GZVM_CREATE_VM, NULL);
    if (ret < 0) {
        gz_report("gzvm: GZVM_CREATE_VM failed: %s (errno=%d)",
                     strerror(errno), errno);
        close(s->fd);
        return -1;
    }
    s->vmfd = ret;
    return 0;
}

static int gzvm_create_vgic_devices(GZVMState *s)
{
    int ret;

    ret = gzvm_create_vgic_device(s, GZVM_DEV_TYPE_ARM_VGIC_V3_DIST,
                                  0x08000000ULL, 0x10000,
                                  &s->gic_dist_base, "VGIC_DIST");
    if (ret) {
        close(s->vmfd);
        s->vmfd = -1;
        close(s->fd);
        s->fd = -1;
        return -1;
    }

    ret = gzvm_create_vgic_device(s, GZVM_DEV_TYPE_ARM_VGIC_V3_REDIST,
                                  0x080A0000ULL, 0x20000ULL,
                                  &s->gic_redist_base, "VGIC_REDIST");
    if (ret) {
        close(s->vmfd);
        s->vmfd = -1;
        close(s->fd);
        s->fd = -1;
        return -1;
    }

    return 0;
}

int gzvm_create_vm(void)
{
    AccelState *accel = current_accel();
    GZVMState *s;

    if (!accel) {
        return -1;
    }
    s = GZVM_STATE(accel);

    if (gzvm_open_device(s)) {
        return -1;
    }

    s->nr_slots_allocated = 32;
    s->slots = g_new0(gzvm_slot, s->nr_slots_allocated);
    s->sorted_ids = g_new0(gint, s->nr_slots_allocated);
    qemu_mutex_init(&s->slots_lock);
    s->nr_active_slots = 0;
    for (uint32_t i = 0; i < s->nr_slots_allocated; ++i) {
        s->slots[i].id = i;
    }
    QSIMPLEQ_INIT(&s->transaction_add);
    QSIMPLEQ_INIT(&s->transaction_del);

    gzvm_install_sigsegv_handler();
    memory_listener_register(&gzvm_memory_listener, &address_space_memory);
    memory_listener_register(&gzvm_ioeventfd_listener, &address_space_memory);
    memory_listener_register(&gzvm_io_listener, &address_space_io);

    return gzvm_create_vgic_devices(s);
}
