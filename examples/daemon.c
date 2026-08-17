/**
 * @file daemon.c
 * @brief A two-boat fleet stepping at 125 ms under a coordinator.
 */
#define _POSIX_C_SOURCE 200809L

#include <fleet_dt/coordinator.h>
#include <fleet_dt/dte.h>
#include <fleet_dt/feasibility.h>
#include <fleet_dt/tick.h>

#include <stdio.h>
#include <stdlib.h>

#define NBOATS     2
#define QUEUE_CAP  16
#define FRAMES     40
#define FDT_WINDOW 4

/** Radians to degrees, for crossing the mixed units of Table I. */
#define RAD2DEG 57.2957795f

static fdt_state_t g_queues[NBOATS][QUEUE_CAP];
static fdt_state_t g_bt_slots[NBOATS];
static fdt_twin_t  g_twins[NBOATS];
static fdt_feas_t  g_feas;

/** c^t: how spread out the fleet is, and the throttle ceiling that follows. */
typedef struct {
    float spread_deg;
    float throttle_ceiling_pct;
} fleet_ctx_t;

/**
 * delta^e over the n most recent states.
 */
static void demo_delta_e(const fdt_queue_t *q, size_t n, const fdt_input_t *in,
                         const fdt_goal_t *g_prev, fdt_state_t *out,
                         void *ctx, void *fleet_ctx)
{
    (void)g_prev;
    (void)ctx;
    (void)fleet_ctx;

    const fdt_state_t *newest = fdt_window_at(q, n, 0);
    const fdt_state_t *oldest = fdt_window_at(q, n, n - 1);
    *out = (newest != NULL) ? *newest : (fdt_state_t){0};

    const float rate = (in != NULL) ? in->wz_rps : 0.0f;
    const float dt_s = (float)FDT_TICK_NS / 1e9f;

    out->yaw_rate_rps = rate;

    out->yaw_deg += rate * dt_s * RAD2DEG;

    /* The term depth buys: mean yaw change per frame across the window. */
    if (newest != NULL && oldest != NULL && n > 1) {
        out->pitch_deg = (newest->yaw_deg - oldest->yaw_deg) /
                         (float)(n - 1);
    }
}

static void demo_pi(const fdt_state_t *b, const fdt_goal_t *g_now,
                    fdt_actuation_t *out, void *ctx, void *fleet_ctx)
{
    (void)ctx;

    const fleet_ctx_t *fc = (const fleet_ctx_t *)fleet_ctx;
    out->cage_rad     = (g_now->yaw_deg - b->yaw_deg) * 0.01f;
    out->throttle_pct = (fc != NULL) ? fc->throttle_ceiling_pct : 100.0f;
}

/** The coordinator's c^t: the paper's example is inter-vessel distance. */
static void demo_ctx(const fdt_store_t *bt, void *fleet_ctx, void *user)
{
    (void)user;

    fleet_ctx_t *fc = (fleet_ctx_t *)fleet_ctx;
    float lo = 1e30f;
    float hi = -1e30f;

    for (size_t k = 0; k < fdt_store_size(bt); k++) {
        const float y = fdt_store_get(bt, k)->yaw_deg;
        if (y < lo) { lo = y; }
        if (y > hi) { hi = y; }
    }
    fc->spread_deg = hi - lo;

    /* A station-keeping policy would derive the ceiling from that spread. */
    fc->throttle_ceiling_pct = (fc->spread_deg > 20.0f) ? 40.0f : 60.0f;
}

/** The coordinator's goals, which in a real system come from the MCS. */
static void demo_plan(const fdt_store_t *bt, const void *fleet_ctx,
                      fdt_goal_t *goals_out, size_t n, void *user)
{
    (void)bt;
    (void)fleet_ctx;
    (void)user;

    static const float targets[NBOATS] = { 90.0f, 45.0f };
    for (size_t k = 0; k < n && k < NBOATS; k++) {
        goals_out[k].yaw_deg = targets[k];
    }
}

/** A second simulation sharing the DTE tick, as Section III describes. */
static void fluid_stub(size_t tick, void *user)
{
    unsigned *ticks = (unsigned *)user;
    (void)tick;
    (*ticks)++;
}

int main(void)
{
    const fdt_state_t b0 = {0};

    for (size_t k = 0; k < NBOATS; k++) {
        if (fdt_twin_init(&g_twins[k], g_queues[k], QUEUE_CAP,
                          demo_delta_e, demo_pi, NULL) != 0) {
            fprintf(stderr, "twin %zu failed to init\n", k);
            return EXIT_FAILURE;
        }
        for (int s = 0; s < FDT_WINDOW; s++) {
            if (fdt_twin_seed(&g_twins[k], &b0) != 0) {
                fprintf(stderr, "twin %zu failed to seed\n", k);
                return EXIT_FAILURE;
            }
        }
    }

    fleet_ctx_t fc = { .spread_deg = 0.0f, .throttle_ceiling_pct = 60.0f };

    fdt_fleet_t fleet;
    fdt_store_t bt;
    fdt_coord_t coord;
    if (fdt_fleet_init(&fleet, g_twins, NBOATS, &fc) != 0 ||
        fdt_store_init(&bt, g_bt_slots, NBOATS) != 0 ||
        fdt_coord_init(&coord, &fleet, &bt, demo_ctx, demo_plan, NULL) != 0) {
        fprintf(stderr, "fleet wiring failed\n");
        return EXIT_FAILURE;
    }

    /* The DTE, with a second simulation on the same tick. */
    unsigned fluid_ticks = 0;
    fdt_sim_slot_t sim_slots[2];
    fdt_dte_t dte;
    if (fdt_dte_init(&dte, sim_slots, 2) != 0 ||
        fdt_dte_register(&dte, "fluid", fluid_stub, &fluid_ticks) != 0) {
        fprintf(stderr, "DTE wiring failed\n");
        return EXIT_FAILURE;
    }

    fdt_input_t     ins[NBOATS] = {0};
    fdt_goal_t      gp[NBOATS]  = {0};
    fdt_goal_t      gn[NBOATS]  = {0};
    fdt_state_t     bs[NBOATS];
    fdt_actuation_t as[NBOATS];

    ins[0].wz_rps = 0.20f;
    ins[1].wz_rps = 0.35f;

    fdt_feas_init(&g_feas, FDT_TICK_NS);

    fdt_tick_t tk;
    fdt_tick_start(&tk, FDT_TICK_NS);

    printf("%-4s %-28s %-28s %8s\n",
           "k", "boat 0 (yaw / trend / cage)", "boat 1 (yaw / trend / cage)",
           "spread");

    for (int frame = 0; frame < FRAMES; frame++) {
        fdt_feas_begin(&g_feas);
        const int rc = fdt_coord_step(&coord, ins, FDT_WINDOW,
                                      gp, gn, bs, as);
        fdt_feas_end(&g_feas);

        if (rc != 0) {
            fprintf(stderr, "fleet step failed at frame %d\n", frame);
            return EXIT_FAILURE;
        }

        (void)fdt_dte_tick(&dte);

        printf("%-4d %7.2f %7.3f %+7.3f      %7.2f %7.3f %+7.3f      %7.3f\n",
               frame,
               (double)bs[0].yaw_deg, (double)bs[0].pitch_deg,
               (double)as[0].cage_rad,
               (double)bs[1].yaw_deg, (double)bs[1].pitch_deg,
               (double)as[1].cage_rad,
               (double)fc.spread_deg);

        fdt_tick_wait(&tk);
    }

    printf("\nframes=%d  overruns=%llu  feasible=%d  worst_delta=%.1f us\n",
           FRAMES,
           (unsigned long long)fdt_tick_overruns(&tk),
           fdt_feas_ok(&g_feas),
           (double)fdt_feas_worst_ns(&g_feas) / 1e3);
    printf("queue depth=%zu of %d  queue bytes per vessel=%zu (48d)\n",
           fdt_twin_depth(&g_twins[0]), QUEUE_CAP,
           fdt_queue_bytes(QUEUE_CAP));
    printf("DTE ticks=%zu  parallel simulations=%zu  fluid stub ran %u times\n",
           fdt_dte_ticks(&dte), fdt_dte_count(&dte), fluid_ticks);

    return EXIT_SUCCESS;
}
