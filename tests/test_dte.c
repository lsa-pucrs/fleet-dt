/**
 * @file test_dte.c
 * @brief Parallel simulations sharing one tick.
 *
 * Claim covered: C7, Section I feature (ii). See docs/spec/paper-claims.md.
 */
#include <fleet_dt/dte.h>

#include <assert.h>
#include <stdio.h>
#include <string.h>

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
    /* Section III's own list of what shares the DTE: WeBots, a fluid
     * simulator, and ML applications composing the Application Space. */
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

int main(void)
{
    test_init_and_registration();
    test_shared_tick();

    printf("test_dte: ok\n");
    return 0;
}
