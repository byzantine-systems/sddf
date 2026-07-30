/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdint.h>
#include <os/sddf.h>
#include <sddf/timer/protocol.h>
#include <sddf/timer/config.h>
#include <sddf/timer/timer_driver.h>
#include <sddf/util/util.h>
#include <sddf/util/printf.h>
#include <sddf/util/udivmodti4.h>
#include <sddf/resources/device.h>

#if !(CONFIG_EXPORT_PCNT_USER && CONFIG_EXPORT_PTMR_USER)
#error "ARM generic timer is not exported by seL4"
#endif

static uint64_t timer_freq;

#define MAX_TIMEOUTS SDDF_TIMER_MAX_CLIENTS

#define GENERIC_TIMER_ENABLE (1 << 0)
#define GENERIC_TIMER_IMASK  (1 << 1)
#define GENERIC_TIMER_STATUS (1 << 2)
#define LOW_WORD(x) (x & 0xffffffffffffffff)
#define HIGH_WORD(x) (x >> 64)

#define COPROC_WRITE_WORD(R, W) asm volatile ("msr " R  ", %0" :: "r"(W))
#define COPROC_READ_WORD(R, W)  asm volatile ("mrs %0, " R : "=r" (W))
#define COPROC_WRITE_64(R, W)   COPROC_WRITE_WORD(R,W)
#define COPROC_READ_64(R, W)    COPROC_READ_WORD(R,W)

/* control reigster for the el1 physical timer */
#define CNTP_CTL "cntp_ctl_el0"
/* holds the compare value for the el1 physical timer */
#define CNTP_CVAL "cntp_cval_el0"
/* holds the 64-bit physical count value */
#define CNTPCT "cntpct_el0"
/* frequency of the timer */
#define CNTFRQ "cntfrq_el0"

__attribute__((__section__(".device_resources"), retain, used)) device_resources_t device_resources;

static inline uint64_t get_ticks(void)
{
    uint64_t time;
    COPROC_READ_64(CNTPCT, time);
    return time;
}

static inline void generic_timer_set_compare(uint64_t ticks)
{
    COPROC_WRITE_64(CNTP_CVAL, ticks);
}

static inline uint32_t generic_timer_get_freq(void)
{
    uintptr_t freq;
    COPROC_READ_WORD(CNTFRQ, freq);
    return (uint32_t)freq;
}

static inline uint32_t generic_timer_read_ctrl(void)
{
    uintptr_t ctrl;
    COPROC_READ_WORD(CNTP_CTL, ctrl);
    return ctrl;
}

static inline void generic_timer_write_ctrl(uintptr_t ctrl)
{
    COPROC_WRITE_WORD(CNTP_CTL, ctrl);
}

static inline void generic_timer_or_ctrl(uintptr_t bits)
{
    uintptr_t ctrl = generic_timer_read_ctrl();
    generic_timer_write_ctrl(ctrl | bits);
}

static inline void generic_timer_enable(void)
{
    generic_timer_or_ctrl(GENERIC_TIMER_ENABLE);
}

static inline void generic_timer_disable(void)
{
    generic_timer_or_ctrl(~GENERIC_TIMER_ENABLE);
}

void set_timeout(uint64_t timeout)
{
    generic_timer_set_compare(ns_to_ticks(timeout, timer_freq));
}

static uint64_t timeouts[MAX_TIMEOUTS];

/* Chrysopolis (driver restart): false only on a COLD boot. A warm restart does
 * not re-zero .bss, so this stays true and lets init() tell cold boot apart
 * from a restart. See the lost-timeout notification in init(). */
static bool initialised;

static void process_timeouts(uint64_t curr_time)
{
    for (int i = 0; i < MAX_TIMEOUTS; i++) {
        if (timeouts[i] <= curr_time) {
            sddf_notify(i);
            timeouts[i] = UINT64_MAX;
        }
    }

    uint64_t next_timeout = UINT64_MAX;
    for (int i = 0; i < MAX_TIMEOUTS; i++) {
        if (timeouts[i] < next_timeout) {
            next_timeout = timeouts[i];
        }
    }

    if (next_timeout != UINT64_MAX) {
        set_timeout(next_timeout);
    }
}

void init()
{
    /* Chrysopolis (driver restart) DIAGNOSTIC, kept commented out. Proved that
     * this PD DOES fully re-execute init() after microkit_pd_restart (both this
     * and the matching |done print at the end of init appeared after every
     * restart), ruling out "the restart never brought the PD back". Re-enable
     * when investigating restart behaviour. */
    /* sddf_dprintf("TIMER|init|enter\n"); */

    assert(device_resources_check_magic(&device_resources));
    assert(device_resources.num_irqs == 1);
    assert(device_resources.num_regions == 0);

    /*
     * Chrysopolis (driver restart): tell clients their timeouts were lost.
     *
     * The loop below discards every armed timeout, and the notification each
     * client is waiting for will therefore never fire. Clients cannot detect
     * this on their own: from their side a lost timeout is indistinguishable
     * from one that has not fired yet, so they wait forever. Worse, a client
     * whose periodic tick re-arms itself from its own timeout handler (as the
     * lwIP service tick in beam_server does) loses the very thing that would
     * have re-armed it, so it can never recover no matter how long it waits.
     *
     * We can identify exactly who to tell because a warm restart does NOT
     * re-zero .bss: timeouts[] still holds the previous instance's entries at
     * this point, and its index IS the client's channel id (see protected(),
     * which stores timeouts[ch], and process_timeouts(), which notifies by the
     * same index). So notify precisely the channels that had a timeout armed.
     *
     * The `initialised` guard is essential rather than cosmetic: on a COLD boot
     * timeouts[] is .bss-zeroed, and 0 != UINT64_MAX, so every one of the 64
     * slots would look armed and we would signal channels that are not wired to
     * anything, faulting the driver at boot. `initialised` is itself .bss, so it
     * is false exactly once (cold boot) and true on every warm restart, which is
     * the distinction we need.
     *
     * Clients need no new code: this arrives on the same channel a fired
     * timeout would, so an existing "my timeout fired" handler re-arms
     * naturally. Upstream candidate.
     */
    if (initialised) {
        for (int i = 0; i < MAX_TIMEOUTS; i++) {
            if (timeouts[i] != UINT64_MAX) {
                sddf_dprintf("TIMER|init|lost-timeout|ch=%d\n", i);
                sddf_notify(i);
            }
        }
    }
    initialised = true;

    for (int i = 0; i < MAX_TIMEOUTS; i++) {
        timeouts[i] = UINT64_MAX;
    }

    generic_timer_set_compare(UINT64_MAX);
    generic_timer_enable();
    timer_freq = generic_timer_get_freq();

    /*
     * Chrysopolis (driver restart): re-arm the IRQ.
     *
     * This driver acks its interrupt ONLY in notified() (sddf_deferred_irq_ack).
     * If an instance dies between the kernel delivering the timer IRQ and that
     * ack, the interrupt stays masked and seL4 never delivers timer IRQ again.
     *
     * That is fatal rather than merely degraded, because the IRQ is the ONLY way
     * a timeout is ever delivered: protected()/SDDF_TIMER_SET_TIMEOUT just
     * programs CNTP_CVAL, and it is the resulting interrupt that runs
     * process_timeouts() and notifies the client. So a lost ack means no timeout
     * EVER fires again, no matter how correctly clients re-arm. Clients that
     * block on a timeout then hang forever, which takes the whole system down,
     * not just the clock.
     *
     * Ordering: generic_timer_set_compare(UINT64_MAX) above pushes the compare
     * out of reach, so the level-triggered line is deasserted before we ack.
     * Acking when nothing is pending is harmless.
     *
     * The virtio net driver already acks at the end of its init(); the serial
     * driver needed the same fix for the same reason once it became restartable.
     *
     * Upstream candidate.
     */
    sddf_irq_ack(device_resources.irqs[0].id);

    /* Chrysopolis (driver restart) DIAGNOSTIC, kept commented out. Reaching
     * here means init() completed and libmicrokit is about to enter its
     * seL4_Recv loop, i.e. the PD is ready to serve PPCs again. Observed after
     * every restart, with the frequency re-read correctly. */
    /* sddf_dprintf("TIMER|init|done|freq=%lu\n", (unsigned long)timer_freq); */
}

void notified(sddf_channel ch)
{
    /* Chrysopolis (driver restart): tolerate an unexpected channel instead of
     * asserting. The upstream assert turns any stray notification into a PD
     * fault, which for a RESTARTABLE driver is a restart loop rather than a
     * clean failure: root restarts it, it faults again on the next stray
     * notification, and the budget burns down until root stops the timer
     * permanently. Logging and ignoring keeps a spurious signal survivable.
     * Upstream candidate. */
    if (ch != device_resources.irqs[0].id) {
        sddf_dprintf("TIMER|notified|unexpected-channel=%u\n", (unsigned)ch);
        return;
    }

    /* Chrysopolis (driver restart) DIAGNOSTIC, kept commented out. Measured
     * 1298 IRQs before a restart and 1819 after, i.e. the interrupt path is
     * fully alive across a restart. Re-enable to re-measure. */
    /* sddf_dprintf("TIMER|irq\n"); */

    sddf_deferred_irq_ack(ch);

    generic_timer_set_compare(UINT64_MAX);
    uint64_t curr_time = ticks_to_ns(get_ticks(), timer_freq);
    process_timeouts(curr_time);
}

seL4_MessageInfo_t protected(sddf_channel ch, seL4_MessageInfo_t msginfo)
{
    /* Chrysopolis (driver restart) DIAGNOSTIC, kept commented out. Proved that
     * client PPCs ARE delivered to and served by the restarted driver: both
     * label=0 (GET_TIME) and label=1 (SET_TIMEOUT) flow freely afterwards. That
     * ruled out a broken endpoint/reply path, which a hung seL4_Call on the
     * client side otherwise looks exactly like. */
    /* sddf_dprintf("TIMER|ppc|label=%lu\n", (unsigned long)seL4_MessageInfo_get_label(msginfo)); */

    switch (seL4_MessageInfo_get_label(msginfo)) {
    case SDDF_TIMER_GET_TIME: {
        uint64_t time_ns = ticks_to_ns(get_ticks(), timer_freq);
        sddf_set_mr(0, time_ns);
        return seL4_MessageInfo_new(0, 0, 0, 1);
    }
    case SDDF_TIMER_SET_TIMEOUT: {
        uint64_t curr_time = ticks_to_ns(get_ticks(), timer_freq);
        uint64_t offset_us = (uint64_t)(sddf_get_mr(0));
        timeouts[ch] = curr_time + offset_us;
        process_timeouts(curr_time);
        break;
    }
    default:
        sddf_dprintf("TIMER DRIVER|LOG: Unknown request %lu to timer from channel %u\n",
                     seL4_MessageInfo_get_label(msginfo), ch);
        break;
    }

    return seL4_MessageInfo_new(0, 0, 0, 0);
}
