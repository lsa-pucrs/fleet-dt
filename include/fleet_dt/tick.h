#ifndef FLEET_DT_TICK_H
#define FLEET_DT_TICK_H

#include <stdint.h>
#include <time.h>

/**
 * @file tick.h
 * @brief The 125 ms frame pacer of Sections III and IV.
 *
 * Section III: "The DTI updates every 125 ms (this paper); each update in the
 * real world corresponds to a single increment in k."
 * Section IV: "The simulation frequency is 8 Hz, i.e. delta is a hard
 * real-time task with deadline of 125 ms."
 *
 * The pacer is the runtime that drives the frames. It is deliberately not
 * part of the model: equation (1) declares no time field, so no state and no
 * input carries a timestamp, and nothing in transition.h or fleet.h takes a
 * clock argument.
 *
 * @note Requires POSIX.1-2008 clock_nanosleep() with TIMER_ABSTIME, which
 *       glibc provides and Apple libc does not.
 */

/** The frame period of the paper, in nanoseconds: 125 ms, i.e. 8 Hz. */
#define FDT_TICK_NS 125000000L

/**
 * @brief An absolute-deadline pacer.
 *
 * Holding the next deadline rather than measuring elapsed time per frame is
 * what keeps scheduling jitter from accumulating: a frame that wakes late
 * does not push the following frame late too.
 */
typedef struct {
    struct timespec next;     /**< Absolute deadline of the next frame. */
    long            period;   /**< Frame period in nanoseconds, positive. */
    uint64_t        overruns; /**< Frames whose work exceeded the period. */
} fdt_tick_t;

/**
 * @brief Arms the pacer and sets the first deadline one period from now.
 * @param period_ns  Frame period in nanoseconds. A value of zero or less is
 *                   replaced by ::FDT_TICK_NS, so a miswired caller runs at
 *                   the paper's rate rather than spinning.
 */
void fdt_tick_start(fdt_tick_t *tk, long period_ns);

/**
 * @brief Sleeps until the next deadline, then advances it by one period.
 *
 * When the deadline has already passed the call returns at once, counts an
 * overrun, and re-arms from the current time. Re-arming rather than catching
 * up matters: a pacer that tried to replay a backlog of missed deadlines
 * would spin through them at full speed and never recover the schedule.
 *
 * Interruption by a signal resumes the same absolute deadline rather than
 * restarting the interval.
 */
void fdt_tick_wait(fdt_tick_t *tk);

/**
 * @brief Frames that missed their deadline since fdt_tick_start().
 * @return The count, or 0 for a NULL pacer.
 */
uint64_t fdt_tick_overruns(const fdt_tick_t *tk);

#endif /* FLEET_DT_TICK_H */
