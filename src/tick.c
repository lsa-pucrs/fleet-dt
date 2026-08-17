#define _POSIX_C_SOURCE 200809L

#include <fleet_dt/tick.h>

#include <errno.h>
#include <stddef.h>

#define NS_PER_S 1000000000L

/** Advances a timespec by a nanosecond delta, normalising the carry. */
static void advance(struct timespec *ts, long ns)
{
    ts->tv_nsec += ns;
    while (ts->tv_nsec >= NS_PER_S) {
        ts->tv_nsec -= NS_PER_S;
        ts->tv_sec  += 1;
    }
}

/** True when @p now is strictly past @p deadline. */
static int passed(const struct timespec *deadline, const struct timespec *now)
{
    if (now->tv_sec != deadline->tv_sec) {
        return now->tv_sec > deadline->tv_sec;
    }
    return now->tv_nsec > deadline->tv_nsec;
}

void fdt_tick_start(fdt_tick_t *tk, long period_ns)
{
    if (tk == NULL) {
        return;
    }
    tk->period   = (period_ns > 0) ? period_ns : FDT_TICK_NS;
    tk->overruns = 0;
    clock_gettime(CLOCK_MONOTONIC, &tk->next);
    advance(&tk->next, tk->period);
}

void fdt_tick_wait(fdt_tick_t *tk)
{
    if (tk == NULL || tk->period <= 0) {
        return;
    }

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    if (passed(&tk->next, &now)) {
        /* The frame's work outran its budget. Count it and re-arm from now,
         * rather than sleeping a negative interval or replaying a backlog. */
        tk->overruns++;
        tk->next = now;
        advance(&tk->next, tk->period);
        return;
    }

    /* Absolute deadline: jitter inside one frame does not shift the next. */
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tk->next, NULL)
           == EINTR) {
        /* Interrupted by a signal; the deadline is unchanged, so resume. */
    }
    advance(&tk->next, tk->period);
}

uint64_t fdt_tick_overruns(const fdt_tick_t *tk)
{
    return (tk == NULL) ? 0 : tk->overruns;
}
