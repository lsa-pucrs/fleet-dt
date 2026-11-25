/**
 * @file test_dte.c
 * @brief Parallel simulations sharing one tick.
 */
#define _POSIX_C_SOURCE 200809L

#include <fleet_dt/dte.h>

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/** Records the ticks one simulation observed, in order. */
typedef struct {
    size_t seen[8];
    size_t n;
} recorder_t;

static void record(size_t tick, void *user)
{
    recorder_t *r = (recorder_t *)user;
    if (r->n < 8) {
        r->seen[r->n++] = tick;
    }
}

static void test_init_and_registration(void)
{
    fdt_sim_slot_t slots[3];
    fdt_dte_t dte;
    recorder_t a = {0};

    assert(fdt_dte_init(&dte, slots, 3) == 0);
    assert(fdt_dte_init(&dte, NULL, 3) == -1);
    assert(fdt_dte_init(&dte, slots, 0) == -1);
    assert(fdt_dte_init(&dte, slots, 3) == 0);

    assert(fdt_dte_count(&dte) == 0);
    assert(fdt_dte_ticks(&dte) == 0);
    assert(fdt_dte_name(&dte, 0) == NULL);

    assert(fdt_dte_register(&dte, "webots", record, &a) == 0);
    assert(fdt_dte_register(&dte, "fluid", NULL, &a) == -1);
    assert(fdt_dte_register(&dte, NULL, record, &a) == -1);
    assert(fdt_dte_count(&dte) == 1);
    assert(strcmp(fdt_dte_name(&dte, 0), "webots") == 0);

    /* The registry fills up rather than overrunning its slots. */
    assert(fdt_dte_register(&dte, "fluid", record, &a) == 0);
    assert(fdt_dte_register(&dte, "detector", record, &a) == 0);
    assert(fdt_dte_register(&dte, "overflow", record, &a) == -1);
    assert(fdt_dte_count(&dte) == 3);

    assert(fdt_dte_tick(NULL) == (size_t)-1);
    assert(fdt_dte_count(NULL) == 0);
}

/** Every simulation sees the same index, in registration order. */
static void test_shared_tick(void)
{
    fdt_sim_slot_t slots[3];
    fdt_dte_t dte;
    recorder_t webots = {0};
    recorder_t fluid = {0};
    recorder_t detector = {0};

    assert(fdt_dte_init(&dte, slots, 3) == 0);
    assert(fdt_dte_register(&dte, "webots", record, &webots) == 0);
    assert(fdt_dte_register(&dte, "fluid", record, &fluid) == 0);
    assert(fdt_dte_register(&dte, "obstacle-detector", record,
                            &detector) == 0);

    for (size_t t = 0; t < 5; t++) {
        assert(fdt_dte_tick(&dte) == t);
    }

    assert(fdt_dte_ticks(&dte) == 5);
    assert(webots.n == 5 && fluid.n == 5 && detector.n == 5);

    for (size_t t = 0; t < 5; t++) {
        /* One index, three simulations: nothing drifts. */
        assert(webots.seen[t] == t);
        assert(fluid.seen[t] == t);
        assert(detector.seen[t] == t);
    }
}

/** A simulation that takes 20 ms of wall time per tick. */
typedef struct {
    size_t seen[8];
    size_t n;
} sleeper_t;

static void sleep_20ms(size_t tick, void *user)
{
    sleeper_t *s = (sleeper_t *)user;
    const struct timespec d = { 0, 20 * 1000 * 1000 };
    nanosleep(&d, NULL);
    if (s->n < 8) {
        s->seen[s->n] = tick;
    }
    s->n++;
}

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

/**
 * Section I, contribution (ii): simulations in parallel in the same DTE.
 * Three 20 ms simulations take 60 ms per tick in sequence and about 20 ms
 * in parallel; every one still sees the same tick index and has finished
 * when the tick returns.
 */
static void test_parallel(void)
{
    fdt_sim_slot_t slots[3];
    fdt_dte_t dte;
    static sleeper_t sims[3];

    assert(fdt_dte_init(&dte, slots, 3) == 0);
    assert(fdt_dte_start_parallel(&dte) == -1); /* nothing registered */
    assert(fdt_dte_register(&dte, "webots", sleep_20ms, &sims[0]) == 0);
    assert(fdt_dte_register(&dte, "fluid", sleep_20ms, &sims[1]) == 0);
    assert(fdt_dte_register(&dte, "ml", sleep_20ms, &sims[2]) == 0);

    double t0 = now_s();
    (void)fdt_dte_tick(&dte);
    (void)fdt_dte_tick(&dte);
    const double seq = (now_s() - t0) / 2.0;

    assert(fdt_dte_start_parallel(&dte) == 0);
    assert(fdt_dte_is_parallel(&dte) == 1);
    assert(fdt_dte_start_parallel(&dte) == -1); /* already running */
    assert(fdt_dte_register(&dte, "late", sleep_20ms, &sims[0]) == -1);

    t0 = now_s();
    for (int k = 0; k < 4; k++) {
        (void)fdt_dte_tick(&dte);
        /* The tick returns only once every simulation has finished it. */
        for (int i = 0; i < 3; i++) {
            assert(sims[i].n == (size_t)(3 + k));
        }
    }
    const double par = (now_s() - t0) / 4.0;
    assert(fdt_dte_stop_parallel(&dte) == 0);
    assert(fdt_dte_stop_parallel(&dte) == -1);
    assert(fdt_dte_is_parallel(&dte) == 0);

    /* Back in sequence after the stop. */
    assert(fdt_dte_tick(&dte) == 6);
    for (int i = 0; i < 3; i++) {
        assert(sims[i].n == 7);
        for (size_t t = 0; t < 7; t++) {
            assert(sims[i].seen[t] == t);
        }
    }

    printf("three 20 ms simulations: %.1f ms per tick in sequence, "
           "%.1f ms in parallel\n", seq * 1e3, par * 1e3);
    assert(seq > 55e-3);
    assert(par < 45e-3);
}

int main(void)
{
    test_init_and_registration();
    test_shared_tick();
    test_parallel();

    printf("test_dte: ok\n");
    return 0;
}
