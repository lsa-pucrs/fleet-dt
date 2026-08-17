/**
 * @file test_tick.c
 * @brief The 125 ms frame period of Sections III and IV.
 *
 * Claim covered: C13. See docs/spec/paper-claims.md.
 */
#define _POSIX_C_SOURCE 200809L

#include <fleet_dt/tick.h>

#include <assert.h>
#include <stdio.h>

/** Milliseconds between two monotonic samples. */
static double elapsed_ms(const struct timespec *a, const struct timespec *b)
{
    return (double)(b->tv_sec - a->tv_sec) * 1e3 +
           (double)(b->tv_nsec - a->tv_nsec) / 1e6;
}

/** The period the paper fixes, and the fallback for a miswired caller. */
static void test_period_constant(void)
{
    assert(FDT_TICK_NS == 125000000L);

    fdt_tick_t dflt;
    fdt_tick_start(&dflt, 0);
    assert(dflt.period == FDT_TICK_NS);
    fdt_tick_start(&dflt, -1);
    assert(dflt.period == FDT_TICK_NS);
}

/**
 * Eight frames at the paper's rate.
 *
 * The bound is loose because the host is not a real-time kernel; what is
 * being checked is that the absolute deadline keeps the mean pinned to the
 * period, not that any single frame is exact.
 */
static void test_measured_period(void)
{
    fdt_tick_t tk;
    fdt_tick_start(&tk, FDT_TICK_NS);
    assert(fdt_tick_overruns(&tk) == 0);

    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    const int frames = 8;
    for (int i = 0; i < frames; i++) {
        fdt_tick_wait(&tk);
    }
    clock_gettime(CLOCK_MONOTONIC, &t1);

    const double per_frame = elapsed_ms(&t0, &t1) / frames;
    printf("measured period: %.3f ms over %d frames (paper: 125 ms)\n",
           per_frame, frames);

    assert(per_frame > 120.0 && per_frame < 135.0);
    assert(fdt_tick_overruns(&tk) == 0);
}

/** Work that outruns the budget is counted, and the pacer re-arms. */
static void test_overrun_accounting(void)
{
    fdt_tick_t fast;
    fdt_tick_start(&fast, 1000000L); /* a 1 ms period, easy to blow */

    struct timespec busy = { .tv_sec = 0, .tv_nsec = 20000000L };
    nanosleep(&busy, NULL);

    fdt_tick_wait(&fast);
    assert(fdt_tick_overruns(&fast) == 1);

    /* Re-armed rather than replaying the backlog: the next wait sleeps a
     * normal period instead of returning immediately nineteen more times. */
    fdt_tick_wait(&fast);
    assert(fdt_tick_overruns(&fast) == 1);

    /* Every accessor is total. */
    assert(fdt_tick_overruns(NULL) == 0);
    fdt_tick_start(NULL, FDT_TICK_NS);
    fdt_tick_wait(NULL);
}

int main(void)
{
    test_period_constant();
    test_measured_period();
    test_overrun_accounting();

    printf("test_tick: ok\n");
    return 0;
}
