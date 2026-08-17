/**
 * @file test_fleet.c
 * @brief Equations (4), (5) and (6): the fleet frame and c^t delivery.
 *
 * Claims covered: C6 (fleet as a DTA of per-vessel DTIs), C17 (homogeneous
 * versus heterogeneous fleets). See docs/spec/paper-claims.md.
 */
#include <fleet_dt/fleet.h>

#include <assert.h>
#include <math.h>
#include <stdio.h>

/** Per-vessel context: what makes two twins behave differently. */
typedef struct {
    float gain;
} vessel_ctx_t;

/**
 * c^t for this test. Note that test_transition.c reads fleet_ctx as a bare
 * float; the two disagree on purpose, because c^t is opaque to the library
 * and every application picks its own type.
 */
typedef struct {
    float ceiling_pct;
    int   seen_by_delta;
    int   seen_by_pi;
} fleet_ctx_t;

/** A delta^e that scales the input by a per-vessel gain. */
static void delta_gain(const fdt_queue_t *q, size_t n, const fdt_input_t *in,
                       const fdt_goal_t *g_prev, fdt_state_t *out,
                       void *ctx, void *fleet_ctx)
{
    (void)g_prev;

    const vessel_ctx_t *vc = (const vessel_ctx_t *)ctx;
    fleet_ctx_t *fc = (fleet_ctx_t *)fleet_ctx;
    if (fc != NULL) {
        fc->seen_by_delta = 1; /* c^t reaches delta^e, not only pi */
    }

    const fdt_state_t *prev = fdt_window_at(q, n, 0);
    *out = (prev != NULL) ? *prev : (fdt_state_t){0};
    out->yaw_deg += ((in != NULL) ? in->wz_rps : 0.0f) * vc->gain;
}

/** A delta^e that holds station: the heterogeneous half of the fleet. */
static void delta_hold(const fdt_queue_t *q, size_t n, const fdt_input_t *in,
                       const fdt_goal_t *g_prev, fdt_state_t *out,
                       void *ctx, void *fleet_ctx)
{
    (void)in;
    (void)g_prev;
    (void)ctx;
    (void)fleet_ctx;

    const fdt_state_t *prev = fdt_window_at(q, n, 0);
    *out = (prev != NULL) ? *prev : (fdt_state_t){0};
}

static void shared_pi(const fdt_state_t *b, const fdt_goal_t *g_now,
                      fdt_actuation_t *out, void *ctx, void *fleet_ctx)
{
    (void)ctx;

    fleet_ctx_t *fc = (fleet_ctx_t *)fleet_ctx;
    if (fc != NULL) {
        fc->seen_by_pi = 1;
    }
    out->cage_rad     = g_now->yaw_deg - b->yaw_deg;
    out->throttle_pct = (fc != NULL) ? fc->ceiling_pct : 100.0f;
}

/** Binding guards and the accessors. */
static void test_init_guards(void)
{
    fdt_state_t st[2][8];
    fdt_twin_t  twins[2];
    vessel_ctx_t vc[2] = { { .gain = 1.0f }, { .gain = 2.0f } };
    fleet_ctx_t  fc = { .ceiling_pct = 60.0f, 0, 0 };
    fdt_fleet_t  f;

    assert(fdt_twin_init(&twins[0], st[0], 8, delta_gain, shared_pi,
                         &vc[0]) == 0);
    assert(fdt_twin_init(&twins[1], st[1], 8, delta_hold, shared_pi,
                         &vc[1]) == 0);

    assert(fdt_fleet_init(&f, twins, 2, &fc) == 0);
    assert(fdt_fleet_init(&f, NULL, 2, &fc) == -1);
    assert(fdt_fleet_init(&f, twins, 0, &fc) == -1);
    assert(fdt_fleet_init(NULL, twins, 2, &fc) == -1);

    assert(fdt_fleet_size(&f) == 2);
    assert(fdt_fleet_size(NULL) == 0);
    assert(fdt_fleet_twin(&f, 0) == &twins[0]);
    assert(fdt_fleet_twin(&f, 1) == &twins[1]);
    assert(fdt_fleet_twin(&f, 2) == NULL);
    assert(fdt_fleet_twin(NULL, 0) == NULL);
}

/** The fleet advances as one frame, or not at all. */
static void test_all_or_nothing(void)
{
    fdt_state_t st[2][8];
    fdt_twin_t  twins[2];
    vessel_ctx_t vc[2] = { { .gain = 1.0f }, { .gain = 2.0f } };
    fleet_ctx_t  fc = { .ceiling_pct = 60.0f, 0, 0 };
    fdt_fleet_t  f;

    assert(fdt_twin_init(&twins[0], st[0], 8, delta_gain, shared_pi,
                         &vc[0]) == 0);
    assert(fdt_twin_init(&twins[1], st[1], 8, delta_hold, shared_pi,
                         &vc[1]) == 0);
    assert(fdt_fleet_init(&f, twins, 2, &fc) == 0);

    fdt_input_t ins[2]   = { { .wz_rps = 1.0f }, { .wz_rps = 1.0f } };
    fdt_goal_t  goals[2] = { { .yaw_deg = 10.0f }, { .yaw_deg = 20.0f } };
    fdt_state_t bs[2];
    fdt_actuation_t as[2];

    /* Nobody seeded: nobody steps. */
    assert(fdt_fleet_step(&f, ins, goals, goals, 1, bs, as) == -1);

    /* One seeded, one not: the seeded vessel must stay untouched. */
    fdt_state_t b0 = {0};
    assert(fdt_twin_seed(&twins[0], &b0) == 0);
    assert(fdt_fleet_step(&f, ins, goals, goals, 1, bs, as) == -1);
    assert(fdt_twin_depth(&twins[0]) == 1);
    assert(fdt_twin_depth(&twins[1]) == 0);

    assert(fdt_twin_seed(&twins[1], &b0) == 0);
    assert(fdt_fleet_step(&f, ins, goals, goals, 1, bs, as) == 0);
    assert(fdt_twin_depth(&twins[0]) == 2);
    assert(fdt_twin_depth(&twins[1]) == 2);

    /* A window wider than the shared history rejects the whole frame. */
    const size_t before = fdt_twin_depth(&twins[0]);
    assert(fdt_fleet_step(&f, ins, goals, goals, 99, bs, as) == -1);
    assert(fdt_twin_depth(&twins[0]) == before);
}

/** C17: per-vessel delta^e, and c^t reaching both halves of the frame. */
static void test_heterogeneous_and_context(void)
{
    fdt_state_t st[2][8];
    fdt_twin_t  twins[2];
    vessel_ctx_t vc[2] = { { .gain = 1.0f }, { .gain = 2.0f } };
    fleet_ctx_t  fc = { .ceiling_pct = 60.0f, 0, 0 };
    fdt_fleet_t  f;
    fdt_state_t  b0 = {0};

    assert(fdt_twin_init(&twins[0], st[0], 8, delta_gain, shared_pi,
                         &vc[0]) == 0);
    assert(fdt_twin_init(&twins[1], st[1], 8, delta_hold, shared_pi,
                         &vc[1]) == 0);
    assert(fdt_twin_seed(&twins[0], &b0) == 0);
    assert(fdt_twin_seed(&twins[1], &b0) == 0);
    assert(fdt_fleet_init(&f, twins, 2, &fc) == 0);

    fdt_input_t ins[2]   = { { .wz_rps = 1.0f }, { .wz_rps = 1.0f } };
    fdt_goal_t  goals[2] = { { .yaw_deg = 10.0f }, { .yaw_deg = 20.0f } };
    fdt_state_t bs[2];
    fdt_actuation_t as[2];

    assert(fdt_fleet_step(&f, ins, goals, goals, 1, bs, as) == 0);

    /* Indexed delta^e: vessel 0 integrates, vessel 1 holds. Same fleet, same
     * frame, different dynamics — no extra machinery required. */
    assert(fabsf(bs[0].yaw_deg - 1.0f) < 1e-6f);
    assert(fabsf(bs[1].yaw_deg - 0.0f) < 1e-6f);

    /* c^t of equation (4) reaches both delta^e and pi. */
    assert(fc.seen_by_delta == 1);
    assert(fc.seen_by_pi == 1);
    assert(fabsf(as[0].throttle_pct - 60.0f) < 1e-6f);

    /* Per-vessel goals are not shared: each pi sees its own. */
    assert(fabsf(as[0].cage_rad - (10.0f - bs[0].yaw_deg)) < 1e-6f);
    assert(fabsf(as[1].cage_rad - (20.0f - bs[1].yaw_deg)) < 1e-6f);

    /* A NULL input array is the no-fresh-telemetry frame, not an error. */
    assert(fdt_fleet_step(&f, NULL, goals, goals, 1, bs, as) == 0);
    assert(fabsf(bs[0].yaw_deg - 1.0f) < 1e-6f);

    /* Equation (6): Delta^e at a depth greater than one. */
    assert(fdt_fleet_step(&f, ins, goals, goals, 3, bs, as) == 0);
}

int main(void)
{
    test_init_guards();
    test_all_or_nothing();
    test_heterogeneous_and_context();

    printf("test_fleet: ok\n");
    return 0;
}
