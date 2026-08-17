/**
 * @file test_coordinator.c
 * @brief The B^t store and the coordinator S of Figure 4.
 */
#include <fleet_dt/coordinator.h>

#include <assert.h>
#include <math.h>
#include <stdio.h>

/** c^t for this fleet: the yaw spread, standing in for inter-vessel distance. */
typedef struct {
    float spread_deg;
    int   ctx_calls;
    int   plan_calls;
} coord_ctx_t;

static void delta_integrate(const fdt_queue_t *q, size_t n,
                            const fdt_input_t *in, const fdt_goal_t *g_prev,
                            fdt_state_t *out, void *ctx, void *fleet_ctx)
{
    (void)g_prev;
    (void)ctx;
    (void)fleet_ctx;

    const fdt_state_t *prev = fdt_window_at(q, n, 0);
    *out = (prev != NULL) ? *prev : (fdt_state_t){0};
    out->yaw_deg += (in != NULL) ? in->wz_rps : 0.0f;
}

static void pi_track(const fdt_state_t *b, const fdt_goal_t *g_now,
                     fdt_actuation_t *out, void *ctx, void *fleet_ctx)
{
    (void)ctx;
    (void)fleet_ctx;

    out->cage_rad     = g_now->yaw_deg - b->yaw_deg;
    out->throttle_pct = 50.0f;
}

/** Computes c^t from the store, which is where Section IV puts it. */
static void compute_ctx(const fdt_store_t *bt, void *fleet_ctx, void *user)
{
    (void)user;

    coord_ctx_t *c = (coord_ctx_t *)fleet_ctx;
    float lo = 1e30f;
    float hi = -1e30f;

    for (size_t k = 0; k < fdt_store_size(bt); k++) {
        const float y = fdt_store_get(bt, k)->yaw_deg;
        if (y < lo) {
            lo = y;
        }
        if (y > hi) {
            hi = y;
        }
    }
    c->spread_deg = hi - lo;
    c->ctx_calls++;
}

/** Distributes g^t seeing the c^t computed earlier in the same step. */
static void plan_goals(const fdt_store_t *bt, const void *fleet_ctx,
                       fdt_goal_t *goals_out, size_t n, void *user)
{
    (void)user;

    const coord_ctx_t *c = (const coord_ctx_t *)fleet_ctx;
    for (size_t k = 0; k < n; k++) {
        goals_out[k].yaw_deg = fdt_store_get(bt, k)->yaw_deg + c->spread_deg;
    }
}

/** The store on its own. */
static void test_store(void)
{
    fdt_state_t slots[2];
    fdt_store_t bt;

    assert(fdt_store_init(&bt, slots, 2) == 0);
    assert(fdt_store_init(&bt, NULL, 2) == -1);
    assert(fdt_store_init(&bt, slots, 0) == -1);
    assert(fdt_store_size(&bt) == 2);
    assert(fdt_store_size(NULL) == 0);

    assert(fdt_store_get(&bt, 0)->yaw_deg == 0.0f);

    fdt_state_t s = { .yaw_deg = 7.0f };
    assert(fdt_store_put(&bt, 0, &s) == 0);
    assert(fdt_store_put(&bt, 2, &s) == -1);
    assert(fdt_store_put(&bt, 0, NULL) == -1);
    assert(fabsf(fdt_store_get(&bt, 0)->yaw_deg - 7.0f) < 1e-6f);
    assert(fdt_store_get(&bt, 2) == NULL);
    assert(fdt_store_get(NULL, 0) == NULL);
}

/** Binding refuses a store that does not cover the fleet. */
static void test_coord_init_guards(void)
{
    fdt_state_t st[2][4];
    fdt_twin_t  twins[2];
    fdt_state_t slots[2];
    fdt_state_t one_slot[1];
    fdt_store_t bt, small;
    fdt_fleet_t f;
    fdt_coord_t co;
    coord_ctx_t cc = {0};

    assert(fdt_twin_init(&twins[0], st[0], 4, delta_integrate, pi_track,
                         NULL) == 0);
    assert(fdt_twin_init(&twins[1], st[1], 4, delta_integrate, pi_track,
                         NULL) == 0);
    assert(fdt_fleet_init(&f, twins, 2, &cc) == 0);
    assert(fdt_store_init(&bt, slots, 2) == 0);
    assert(fdt_store_init(&small, one_slot, 1) == 0);

    assert(fdt_coord_init(&co, &f, &bt, compute_ctx, plan_goals, NULL) == 0);
    assert(fdt_coord_init(&co, NULL, &bt, compute_ctx, plan_goals, NULL) == -1);
    assert(fdt_coord_init(&co, &f, NULL, compute_ctx, plan_goals, NULL) == -1);
    assert(fdt_coord_init(&co, &f, &bt, NULL, plan_goals, NULL) == -1);
    assert(fdt_coord_init(&co, &f, &bt, compute_ctx, NULL, NULL) == -1);

    /* A one-slot store for a two-vessel fleet would drop a state silently. */
    assert(fdt_coord_init(&co, &f, &small, compute_ctx, plan_goals,
                          NULL) == -1);
}

/** The four stages of one coordinated frame, in Figure 4's order. */
static void test_coord_step(void)
{
    fdt_state_t st[2][4];
    fdt_twin_t  twins[2];
    fdt_state_t slots[2];
    fdt_store_t bt;
    fdt_fleet_t f;
    fdt_coord_t co;
    coord_ctx_t cc = {0};
    fdt_state_t z = {0};

    assert(fdt_twin_init(&twins[0], st[0], 4, delta_integrate, pi_track,
                         NULL) == 0);
    assert(fdt_twin_init(&twins[1], st[1], 4, delta_integrate, pi_track,
                         NULL) == 0);
    assert(fdt_twin_seed(&twins[0], &z) == 0);
    assert(fdt_twin_seed(&twins[1], &z) == 0);
    assert(fdt_fleet_init(&f, twins, 2, &cc) == 0);
    assert(fdt_store_init(&bt, slots, 2) == 0);
    assert(fdt_coord_init(&co, &f, &bt, compute_ctx, plan_goals, NULL) == 0);

    fdt_input_t ins[2] = { { .wz_rps = 1.0f }, { .wz_rps = 3.0f } };
    fdt_goal_t  gp[2]  = {0};
    fdt_goal_t  gn[2]  = {0};
    fdt_state_t bs[2];
    fdt_actuation_t as[2];

    assert(fdt_coord_step(&co, ins, 1, gp, gn, bs, as) == 0);

    /* Stage 2: the store holds this frame's B^t. */
    assert(fabsf(fdt_store_get(&bt, 0)->yaw_deg - 1.0f) < 1e-6f);
    assert(fabsf(fdt_store_get(&bt, 1)->yaw_deg - 3.0f) < 1e-6f);

    /* Stage 3: c^t computed once, from those states. */
    assert(cc.ctx_calls == 1);
    assert(fabsf(cc.spread_deg - 2.0f) < 1e-6f);

    /* Stage 4: g^t distributed in the same step, already seeing c^t. */
    assert(fabsf(gn[0].yaw_deg - 3.0f) < 1e-6f);
    assert(fabsf(gn[1].yaw_deg - 5.0f) < 1e-6f);

    assert(fdt_coord_step(&co, ins, 1, gp, gn, bs, as) == 0);
    assert(fabsf(gp[0].yaw_deg - 3.0f) < 1e-6f);
    assert(cc.ctx_calls == 2);

    /* A failed fleet step leaves the store and the goals untouched. */
    const float spread_before = cc.spread_deg;
    const float goal_before   = gn[0].yaw_deg;
    assert(fdt_coord_step(&co, ins, 99, gp, gn, bs, as) == -1);
    assert(cc.spread_deg == spread_before);
    assert(gn[0].yaw_deg == goal_before);
    assert(cc.ctx_calls == 2);
}

int main(void)
{
    test_store();
    test_coord_init_guards();
    test_coord_step();

    printf("test_coordinator: ok\n");
    return 0;
}
