#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/thread.h"
#include "hw/boards.h"
#include "hw/core/cpu.h"
#include "system/accel-ops.h"
#include "system/cpus.h"
#include "system/gzvm.h"
#include "system/gzvm_int.h"
#include "gzvm-internal.h"
#include "qapi/error.h"

bool gzvm_allowed;

/*
 * Whether to offer VIRTIO_RING_F_EVENT_IDX to guests.  Off under gzvm; set
 * GZVM_EVENT_IDX=on to offer it anyway, which reproduces the hang below.
 *
 * Both of virtio's split-ring handshakes have two forms, and which pair gets
 * negotiated is decided by this one feature bit:
 *
 *   guest -> host kick    EVENT_IDX: vring_need_event(used->avail_event, ...),
 *                                    an index *we* wrote into guest memory
 *                         flags:     !(used->flags & VRING_USED_F_NO_NOTIFY)
 *
 *   host -> guest notify  EVENT_IDX: vring_need_event(avail->used_event, ...),
 *                                    an index the *guest* wrote
 *                         flags:     !(avail->flags & VRING_AVAIL_F_NO_INTERRUPT)
 *
 * The flag forms carry their own recovery.  Whoever re-enables re-examines the
 * ring afterwards -- virtqueue_enable_cb() in the guest is literally
 * enable_cb_prepare() followed by !virtqueue_poll(), and the device side does the
 * same when it calls virtio_queue_set_notification(vq, 1) and then re-checks
 * virtio_queue_empty().  Read a stale flag and you lose a notification, then the
 * re-poll finds the work anyway.
 *
 * The index forms have those same re-poll steps -- virtqueue_enable_cb() works
 * under EVENT_IDX too, and virtblk_done() loops on it -- so one lost index update
 * is not by itself terminal.  What they lack is a conservative default.  A stale
 * flag read errs towards notifying; a stale index read is a comparison against a
 * number that may be arbitrarily far behind, and vring_need_event() answers "no"
 * to most of those.  That leaves a window in which neither side has any reason to
 * look, and the pair settles into a state each of them considers steady.  Both
 * indices live in memory the hypervisor maps rather than we do, and with no ITS
 * in this tree every virtio-pci device is on INTx.
 *
 * A four-cell matrix pinned down the direction.  Two temporary knobs did it, both
 * since removed: one armed a per-device timer that re-polled every virtqueue and
 * synthesised a notify for any that had work waiting, covering guest -> host, and
 * one ignored the guest's used_event so that every completion interrupted,
 * covering host -> guest.  All four cells ran on one binary with INTx already
 * carried by an irqfd, so that nothing de-duplicates between virtio_notify() and
 * the hypervisor, three or four times each at -smp 8 with EVENT_IDX on:
 *
 *   neither knob          hangs
 *   re-poll at 1 ms       boots
 *   forced notification   hangs
 *   both                  boots, no better than the re-poll alone
 *
 * Synthesising the guest's doorbell repairs it and notifying unconditionally does
 * not, so the lost event is the guest -> host kick: the guest reads a stale
 * used->avail_event, vring_need_event() answers no, and we wait in front of an
 * avail ring we believe is empty.  Forced notification on top of the re-poll buys
 * nothing, which rules the host -> guest half out as a contributor.
 *
 * Three caveats on that matrix, since the knobs are gone and this is the only
 * record left.
 *
 * The re-poll's own hit counter was never evidence, and an earlier version of this
 * comment had to withdraw a verdict drawn from it.  Its first form fired whenever a
 * queue merely looked non-empty, which a run at 1 ms showed to be worthless: it
 * logged eight virtio-blk hits while still inside UEFI, and EDK2 does not
 * negotiate EVENT_IDX at all -- VirtioBlkDxe masks the feature set down to
 * BLK_SIZE | TOPOLOGY | RO | FLUSH | VERSION_1 | IOMMU_PLATFORM -- so those hits
 * could only have been the timer landing between the guest's write to avail->idx
 * and our notify handler running.  The same counts, in the same per-device
 * distribution, showed up in boots that succeeded.  The matrix rests on the
 * differential between cells, not on any count.
 *
 * The final form required two consecutive ticks with work waiting and
 * last_avail_idx unmoved in between, which no sub-millisecond race survives.  It
 * kicked an order of magnitude less often than the form it replaced and was no
 * less stable, so the kicks it did issue were doing real work -- "1 ms merely
 * perturbs timing" does not explain cell 2.  It still over-reported: on
 * receive-direction queues, virtio-net's RX ring and virtio-input's eventq, "work
 * waiting and nothing consumed" is the normal idle state, and those were 24 of the
 * 32 hits in the cell-2 run.  The eight that meant anything were virtio-blk, at an
 * advancing avail_idx, during the systemd phase where the hang lands.
 *
 * And the boots were "almost always" rather than always.  One blind spot is known:
 * if avail->idx itself is stale to us then virtio_queue_empty() returns true, the
 * probe never fires, and a loss of that shape was invisible to the instrument.
 * The interval bore that out -- 1 and 5 ms improved the odds, 10, 50, 100 and 500
 * were level with each other, and none eliminated the hang.  A single repairable
 * lost kick should make a long interval slow rather than fatal, so either
 * something upstream in the guest times out while we dawdle, or there is a second
 * loss that probe could not see.
 *
 * Neither knob was ever a candidate fix, which is why both are gone.  Forced
 * notification throws away every interrupt EVENT_IDX exists to elide, and it would
 * be strange to ship that next to simply withdrawing the feature.  And for the
 * guest to kick it must read avail_event == new - 1, which is exactly what
 * virtio_queue_split_set_notification() already writes -- vring_avail_idx(vq),
 * followed by an smp_mb().  No other value forces a kick out of a stale read;
 * 0xffff, for one, actively suppresses it.  So the only levers were withdrawing
 * the feature or running a permanent 1 ms timer per virtio device, and that timer
 * is not worth it for a feature whose whole purpose is to do less work.  (An early
 * forced-notification test that booted -smp 8 "only occasionally" is not evidence
 * either way: it ran while the level/edge defect was still present, so the extra
 * notifications ran into that bug instead.)
 *
 * Nor is this the guest's barriers being too weak.  Offering
 * VIRTIO_F_ORDER_PLATFORM sets vq->weak_barriers = false in the guest, which on
 * arm64 turns virtio_mb() from dmb(ish) into dsb(sy) and virtio_rmb()/wmb() from
 * dmb(ishld)/dmb(ishst) into dmb(oshld)/dmb(oshst) -- Inner Shareable becomes
 * Outer Shareable or full system.  With that bit and EVENT_IDX both on, -smp 8
 * still hung.  Which is what you would expect either way: a barrier orders writes
 * a CPU has already made, it does not make them propagate sooner, so no barrier
 * the guest can execute changes when our write to avail_event becomes visible to
 * it.
 *
 * Nor is it plain non-coherency, and this is the part worth handing to MediaTek.
 * virtqueue_kick_prepare_split() reads either vring_avail_event() or used->flags
 * after the same virtio_mb() -- same used ring, same page, same memory type, one
 * function, one barrier -- and the flag form is reliable: with EVENT_IDX off,
 * -smp 8 boots every time.  If our stores to that page were simply not arriving,
 * the flag form would break too.  What differs between them is not visibility but
 * how each one fails under lag.  A stale flag read only suppresses if it catches
 * the brief window where NO_NOTIFY is set, which is rare and self-correcting.  A
 * stale index read suppresses whenever the value it sees is more than one behind,
 * and avail_event advances monotonically under load, so "more than one behind" is
 * the common case rather than a rare one.  Same lag, wildly different
 * consequences.
 *
 * So the ask is narrower than "host writes to guest RAM are invisible": our stores
 * become visible to the guest with a delay long enough that a continuously
 * advancing counter is routinely read stale, while a rarely toggled flag in the
 * same page survives.  Not reachable from here, and not from the host driver
 * either: drivers/virt/geniezone only hands GZ an address range and never touches
 * guest RAM attributes.  gz.img is a blob.  Withdrawing the bit is the fix.
 *
 * More vCPUs mean more concurrent ring traffic and more chances to lose one,
 * which is why -smp 2 was reliable, 3 and 4 marginal, and 5 and up never
 * finished booting.
 */
bool gzvm_event_idx_allowed(void)
{
    static int allowed = -1;

    if (allowed < 0) {
        const char *env = getenv("GZVM_EVENT_IDX");

        allowed = env && (!strcmp(env, "on") || !strcmp(env, "1"));
        if (allowed) {
            gz_report("gzvm: offering VIRTIO_RING_F_EVENT_IDX; expect hangs "
                        "at -smp 4 and above");
        }
    }

    return allowed != 0;
}

static int gzvm_init(MachineState *ms)
{
    GZVMState *s = GZVM_STATE(current_accel());

    gzvm_ioctl_set_state(s);

    return gzvm_create_vm();
}

static void gzvm_accel_instance_finalize(Object *obj)
{
    GZVMState *s = GZVM_STATE(obj);
    if (s->fd >= 0) {
        close(s->fd);
    }
    if (s->vmfd >= 0) {
        close(s->vmfd);
    }
    g_free(s->slots);
    g_free(s->sorted_ids);
}

static void gzvm_accel_instance_init(Object *obj)
{
    GZVMState *s = GZVM_STATE(obj);
    s->fd = -1;
    s->vmfd = -1;
    s->slots = NULL;
    s->sorted_ids = NULL;
}

static void gzvm_setup_post(MachineState *ms, AccelState *accel)
{
    int r = gzvm_start_vm();
    if (r < 0) {
        gz_report("gzvm: VM start failed");
    }
}

static void gzvm_accel_class_init(ObjectClass *oc, void *data)
{
    AccelClass *ac = ACCEL_CLASS(oc);
    ac->name = "GZVM";
    ac->init_machine = gzvm_init;
    ac->allowed = &gzvm_allowed;
    ac->setup_post = gzvm_setup_post;
}

static const TypeInfo gzvm_accel_type = {
    .name = TYPE_GZVM_ACCEL,
    .parent = TYPE_ACCEL,
    .instance_init = gzvm_accel_instance_init,
    .instance_finalize = gzvm_accel_instance_finalize,
    .class_init = gzvm_accel_class_init,
    .instance_size = sizeof(GZVMState),
};

static void gzvm_type_init(void)
{
    type_register_static(&gzvm_accel_type);
}

type_init(gzvm_type_init);

static void gzvm_start_vcpu_thread(CPUState *cpu)
{
    char thread_name[VCPU_THREAD_NAME_SIZE];
    snprintf(thread_name, VCPU_THREAD_NAME_SIZE, "CPU %d/GZVM",
             cpu->cpu_index);
    qemu_thread_create(cpu->thread, thread_name, gzvm_cpu_thread_fn,
                       cpu, QEMU_THREAD_JOINABLE);
}

static void gzvm_kick_vcpu_thread(CPUState *cpu)
{
    cpus_kick_thread(cpu);
}

static bool gzvm_vcpu_thread_is_idle(CPUState *cpu)
{
    return qatomic_read(&gzvm_vm_stopped);
}

static void gzvm_accel_ops_class_init(ObjectClass *oc, void *data)
{
    AccelOpsClass *ops = ACCEL_OPS_CLASS(oc);
    ops->create_vcpu_thread = gzvm_start_vcpu_thread;
    ops->kick_vcpu_thread = gzvm_kick_vcpu_thread;
    ops->cpu_thread_is_idle = gzvm_vcpu_thread_is_idle;
    ops->synchronize_post_reset = gzvm_cpu_synchronize_post_reset;
    ops->synchronize_post_init = gzvm_cpu_synchronize_post_init;
}

static const TypeInfo gzvm_accel_ops_type = {
    .name = ACCEL_OPS_NAME("gzvm"),
    .parent = TYPE_ACCEL_OPS,
    .class_init = gzvm_accel_ops_class_init,
    .abstract = true,
};

static void gzvm_accel_ops_register_types(void)
{
    type_register_static(&gzvm_accel_ops_type);
}

type_init(gzvm_accel_ops_register_types);
