/**
 * @file bench_jitter.c
 * @brief Frame timing against the 125 ms deadline of Section IV.
 */
#define _POSIX_C_SOURCE 200809L

#include "bench_common.h"
#include "injector/injector.h"

#include <fleet_dt/codec.h>
#include <fleet_dt/coordinator.h>
#include <fleet_dt/envelope.h>
#include <fleet_dt/feasibility.h>
#include <fleet_dt/tick.h>

#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define VESSELS 8
#define CAP     8
#define WINDOW  4
#define FRAMES  80   /* 80 frames at 125 ms is ten seconds of wall clock */

static fdt_state_t     g_queues[VESSELS][CAP];
static fdt_twin_t      g_twins[VESSELS];
static fdt_state_t     g_slots[VESSELS];
static fdt_input_t     g_ins[VESSELS];
static fdt_goal_t      g_gprev[VESSELS];
static fdt_goal_t      g_gnow[VESSELS];
static fdt_state_t     g_bs[VESSELS];
static fdt_actuation_t g_as[VESSELS];
static fdt_input_t     g_decoded[VESSELS];

static double now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e9 + (double)ts.tv_nsec;
}

static void delta_e(const fdt_queue_t *q, size_t n, const fdt_input_t *in,
                    const fdt_goal_t *g_prev, fdt_state_t *out,
                    void *ctx, void *fleet_ctx)
{
    (void)g_prev;
    (void)ctx;
    (void)fleet_ctx;

    const fdt_state_t *newest = fdt_window_at(q, n, 0);
    *out = (newest != NULL) ? *newest : (fdt_state_t){0};

    const float rate = (in != NULL) ? in->wz_rps : 0.0f;
    out->yaw_rate_rps = rate;
    out->yaw_deg += rate * ((float)FDT_TICK_NS / 1e9f) * 57.2957795f;
}

static void pi_track(const fdt_state_t *b, const fdt_goal_t *g_now,
                     fdt_actuation_t *out, void *ctx, void *fleet_ctx)
{
    (void)ctx;
    (void)fleet_ctx;

    out->cage_rad     = (g_now->yaw_deg - b->yaw_deg) * 0.01f;
    out->throttle_pct = 55.0f;
}

static void ctx_fn(const fdt_store_t *bt, void *fleet_ctx, void *user)
{
    (void)bt;
    (void)fleet_ctx;
    (void)user;
}

static void plan_fn(const fdt_store_t *bt, const void *fleet_ctx,
                    fdt_goal_t *goals_out, size_t n, void *user)
{
    (void)bt;
    (void)fleet_ctx;
    (void)user;

    for (size_t k = 0; k < n; k++) {
        goals_out[k].yaw_deg = 60.0f;
    }
}

static void on_lsdt(const char *topic, const uint8_t *buf, size_t len,
                    void *user)
{
    (void)topic;
    (void)user;

    fdt_env_t env;
    const uint8_t *payload = NULL;
    if (fdt_env_decode(buf, len, &env, &payload) < 0 || payload == NULL) {
        return;
    }
    if (env.kind == FDT_ENV_INPUT && env.vessel < VESSELS) {
        (void)fdt_dec_input(payload, env.payload_len, &g_decoded[env.vessel]);
    }
}

static int cmp_double(const void *a, const void *b)
{
    const double x = *(const double *)a;
    const double y = *(const double *)b;
    return (x < y) ? -1 : (x > y) ? 1 : 0;
}

static double pct(const double *sorted, int n, double p)
{
    int idx = (int)(p * (double)(n - 1));
    if (idx < 0) { idx = 0; }
    if (idx >= n) { idx = n - 1; }
    return sorted[idx];
}

int main(void)
{
    bench_report_t r;
    bench_open(&r, "jitter");
    bench_banner(&r, "fleet-dt frame jitter bench");
    bench_say(&r, "vessels: %d   frames: %d   period: %.0f ms   "
                  "(about %.0f s of wall clock)\n",
              VESSELS, FRAMES, (double)FDT_TICK_NS / 1e6,
              (double)FRAMES * (double)FDT_TICK_NS / 1e9);

    fdt_loop_t loop;
    fdt_transport_t tr = fdt_loop_transport(&loop);
    fdt_inj_t inj;
    fdt_fleet_t fleet;
    fdt_store_t store;
    fdt_coord_t coord;
    fdt_feas_t feas;
    fdt_tick_t tk;
    const fdt_state_t zero = {0};

    assert(fdt_inj_init(&inj, &tr, VESSELS, 8.0, 20260817u) == 0);

    for (unsigned k = 0; k < VESSELS; k++) {
        assert(fdt_twin_init(&g_twins[k], g_queues[k], CAP,
                             delta_e, pi_track, NULL) == 0);
        for (int s = 0; s < WINDOW; s++) {
            assert(fdt_twin_seed(&g_twins[k], &zero) == 0);
        }
        char topic[64];
        assert(fdt_topic(topic, sizeof topic, FDT_TOPIC_LSDT, k) == 0);
        assert(tr.subscribe(tr.self, topic, on_lsdt, NULL) == 0);
    }

    assert(fdt_fleet_init(&fleet, g_twins, VESSELS, NULL) == 0);
    assert(fdt_store_init(&store, g_slots, VESSELS) == 0);
    assert(fdt_coord_init(&coord, &fleet, &store, ctx_fn, plan_fn, NULL) == 0);
    fdt_feas_init(&feas, FDT_TICK_NS);

    static double xs[FRAMES];
    static double period_ms[FRAMES];
    static double deviation_ms[FRAMES];

    fdt_tick_start(&tk, FDT_TICK_NS);
    double prev = now_ns();

    for (int frame = 0; frame < FRAMES; frame++) {
        assert(fdt_inj_tick(&inj) == VESSELS);
        (void)tr.poll(tr.self, 0);

        for (unsigned k = 0; k < VESSELS; k++) {
            g_ins[k] = g_decoded[k];
        }

        fdt_feas_begin(&feas);
        const int rc = fdt_coord_step(&coord, g_ins, WINDOW,
                                      g_gprev, g_gnow, g_bs, g_as);
        fdt_feas_end(&feas);
        assert(rc == 0);

        fdt_tick_wait(&tk);

        const double t = now_ns();
        xs[frame]           = (double)frame;
        period_ms[frame]    = (t - prev) / 1e6;
        deviation_ms[frame] = period_ms[frame] - (double)FDT_TICK_NS / 1e6;
        prev = t;
    }

    static double sorted[FRAMES - 1];
    for (int i = 1; i < FRAMES; i++) {
        sorted[i - 1] = deviation_ms[i] < 0.0 ? -deviation_ms[i]
                                              : deviation_ms[i];
    }
    qsort(sorted, FRAMES - 1, sizeof(double), cmp_double);

    double sum = 0.0;
    for (int i = 1; i < FRAMES; i++) {
        sum += period_ms[i];
    }
    const double mean_period = sum / (double)(FRAMES - 1);

    char buf[64];

    bench_section(&r, "frame period held under a stepping fleet");
    bench_say(&r, "  mean period      : %8.3f ms\n", mean_period);
    bench_say(&r, "  |deviation| p50  : %8.3f ms\n",
              pct(sorted, FRAMES - 1, 0.50));
    bench_say(&r, "  |deviation| p95  : %8.3f ms\n",
              pct(sorted, FRAMES - 1, 0.95));
    bench_say(&r, "  |deviation| p99  : %8.3f ms\n",
              pct(sorted, FRAMES - 1, 0.99));
    bench_say(&r, "  |deviation| worst: %8.3f ms\n", sorted[FRAMES - 2]);
    bench_say(&r, "  pacer overruns   : %8llu\n",
              (unsigned long long)fdt_tick_overruns(&tk));

    snprintf(buf, sizeof buf, "%.3f ms", mean_period);
    bench_compare(&r, "mean frame period", buf, "125 ms, 8 Hz (Sec. IV)",
                  (mean_period > 124.0 && mean_period < 126.0)
                      ? BENCH_OK : BENCH_DIVERGE);

    snprintf(buf, sizeof buf, "%llu",
             (unsigned long long)fdt_tick_overruns(&tk));
    bench_compare(&r, "deadline misses", buf, "delta is a hard real-time task",
                  (fdt_tick_overruns(&tk) == 0) ? BENCH_OK : BENCH_DIVERGE);

    bench_compare(&r, "WeBots 3D stuttering", "not measured here",
                  "no stuttering (Sec. V-A)", BENCH_NA);
    bench_say(&r,
        "  Stuttering is a property of the renderer, so that half of the\n"
        "  Section V-A claim needs WeBots. What this run establishes is the\n"
        "  precondition: the frame clock holds while %d vessels publish,\n"
        "  decode and step underneath it, so nothing upstream of the\n"
        "  renderer is losing the deadline.\n", VESSELS);

    fdt_plot_t p;
    fdt_plot_init(&p, "Frame period under a stepping fleet",
                  "frame", "period (ms)");
    fdt_plot_series(&p, "measured period", xs + 1, period_ms + 1, FRAMES - 1,
                    FDT_PLOT_LINE);
    fdt_plot_hline(&p, 125.0, "125 ms deadline (Sec. IV)");
    fdt_plot_write_svg(&p, "results/jitter.svg");
    fdt_plot_write_csv(&p, "results/jitter.csv");

    fdt_plot_t d;
    fdt_plot_init(&d, "Deviation from the 125 ms deadline",
                  "frame", "deviation (ms)");
    fdt_plot_series(&d, "period minus deadline", xs + 1, deviation_ms + 1,
                    FRAMES - 1, FDT_PLOT_SCATTER);
    fdt_plot_hline(&d, 0.0, "on time");
    fdt_plot_write_svg(&d, "results/jitter_deviation.svg");

    bench_artefacts(&r, "jitter");
    bench_say(&r, "artefacts: results/jitter_deviation.svg\n");
    bench_close(&r);
    tr.close(tr.self);
    return EXIT_SUCCESS;
}
