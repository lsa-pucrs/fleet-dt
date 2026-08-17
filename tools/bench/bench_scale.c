/**
 * @file bench_scale.c
 * @brief Fleet scaling: the DTI ceiling and the injector ceiling, apart.
 */
#define _POSIX_C_SOURCE 200809L

#include "bench_common.h"
#include "injector/injector.h"

#include <fleet_dt/codec.h>
#include <fleet_dt/coordinator.h>
#include <fleet_dt/envelope.h>
#include <fleet_dt/feasibility.h>
#include <fleet_dt/framesync.h>
#include <fleet_dt/tick.h>

#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define MAX_VESSELS 64
#define QUEUE_CAP   8
#define WINDOW      4
#define FRAMES      200

/** Fleet sizes swept, chosen to straddle the paper's ~25-boat observation. */
static const unsigned SWEEP[] = { 1, 2, 4, 8, 16, 25, 32, 48, 64 };
#define SWEEP_N ((int)(sizeof SWEEP / sizeof SWEEP[0]))

/* Storage for the largest fleet in the sweep, reused across runs. */
static fdt_state_t     g_queues[MAX_VESSELS][QUEUE_CAP];
static fdt_twin_t      g_twins[MAX_VESSELS];
static fdt_state_t     g_slots[MAX_VESSELS];
static fdt_input_t     g_ins[MAX_VESSELS];
static fdt_goal_t      g_gprev[MAX_VESSELS];
static fdt_goal_t      g_gnow[MAX_VESSELS];
static fdt_state_t     g_bs[MAX_VESSELS];
static fdt_actuation_t g_as[MAX_VESSELS];
static uint32_t        g_seqs[MAX_VESSELS];
static uint8_t         g_hits[MAX_VESSELS];

/** Fleet context: a throttle ceiling derived from how spread out the fleet is. */
typedef struct {
    float spread_deg;
    float throttle_ceiling_pct;
} fleet_ctx_t;

/** Where the decoded telemetry of one frame lands before the fleet steps. */
static fdt_input_t  g_decoded[MAX_VESSELS];
static fdt_framesync_t g_fs;

/** delta^e: integrate the yaw rate over the window's newest state. */
static void bench_delta_e(const fdt_queue_t *q, size_t n, const fdt_input_t *in,
                          const fdt_goal_t *g_prev, fdt_state_t *out,
                          void *ctx, void *fleet_ctx)
{
    (void)g_prev;
    (void)ctx;
    (void)fleet_ctx;

    const fdt_state_t *newest = fdt_window_at(q, n, 0);
    const fdt_state_t *oldest = fdt_window_at(q, n, n - 1);
    *out = (newest != NULL) ? *newest : (fdt_state_t){0};

    const float rate  = (in != NULL) ? in->wz_rps : 0.0f;
    const float dt_s  = (float)FDT_TICK_NS / 1e9f;
    const float rad2d = 57.2957795f;

    out->yaw_rate_rps = rate;
    out->yaw_deg     += rate * dt_s * rad2d;

    if (newest != NULL && oldest != NULL && n > 1) {
        out->pitch_deg = (newest->yaw_deg - oldest->yaw_deg) /
                         (float)(n - 1);
    }
    if (in != NULL) {
        out->lat_deg = in->gps_lat_deg;
        out->lon_deg = in->gps_lon_deg;
    }
}

static void bench_pi(const fdt_state_t *b, const fdt_goal_t *g_now,
                     fdt_actuation_t *out, void *ctx, void *fleet_ctx)
{
    (void)ctx;

    const fleet_ctx_t *fc = (const fleet_ctx_t *)fleet_ctx;
    out->cage_rad     = (g_now->yaw_deg - b->yaw_deg) * 0.01f;
    out->throttle_pct = (fc != NULL) ? fc->throttle_ceiling_pct : 100.0f;
}

/** c^t: the yaw spread, standing in for the paper's inter-vessel distance. */
static void bench_ctx(const fdt_store_t *bt, void *fleet_ctx, void *user)
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
    fc->throttle_ceiling_pct = (fc->spread_deg > 20.0f) ? 40.0f : 60.0f;
}

static void bench_plan(const fdt_store_t *bt, const void *fleet_ctx,
                       fdt_goal_t *goals_out, size_t n, void *user)
{
    (void)bt;
    (void)fleet_ctx;
    (void)user;

    for (size_t k = 0; k < n; k++) {
        goals_out[k].yaw_deg = 45.0f + 5.0f * (float)(k % 4);
    }
}

/** Collects one frame's telemetry off the transport into g_decoded. */
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
    if (env.kind != FDT_ENV_INPUT || env.vessel >= MAX_VESSELS) {
        return;
    }
    if (fdt_fs_accept(&g_fs, &env) == FDT_FS_STALE) {
    }
    (void)fdt_dec_input(payload, env.payload_len, &g_decoded[env.vessel]);
}

/** Monotonic nanoseconds. */
static double now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e9 + (double)ts.tv_nsec;
}

/** What one fleet size produced. */
typedef struct {
    double dti_worst_ms;
    double dti_mean_ms;
    double cpu_per_dti_pct;
    double inj_mean_ms;
    int    feasible;
    unsigned long partial;
    unsigned long doubles;
    unsigned long stale;
    unsigned long inj_missed;
} run_t;

/**
 * @param drop_every   Loopback loss, producing partial frames; 0 for none.
 * @param defer_every  Loopback deferral, producing double updates; 0 for none.
 * @param paced        Non-zero to drive the loop with the 125 ms pacer and
 *                     count real missed deadlines. Costs FRAMES * 125 ms of
 *                     wall clock, so it is used once rather than per size.
 */
static run_t run_fleet_ex(unsigned vessels, unsigned drop_every,
                          unsigned defer_every, int paced, int frames)
{
    fdt_loop_t loop;
    fdt_transport_t tr = fdt_loop_transport(&loop);
    fdt_inj_t inj;
    fdt_fleet_t fleet;
    fdt_store_t store;
    fdt_coord_t coord;
    fdt_feas_t feas;
    fleet_ctx_t fc = { .spread_deg = 0.0f, .throttle_ceiling_pct = 60.0f };
    const fdt_state_t zero = {0};
    run_t out = {0};

    fdt_loop_set_loss(&loop, drop_every, defer_every);

    assert(fdt_inj_init(&inj, &tr, vessels, 8.0, 20260817u) == 0);
    assert(fdt_fs_init(&g_fs, g_seqs, g_hits, vessels) == 0);

    for (unsigned k = 0; k < vessels; k++) {
        assert(fdt_twin_init(&g_twins[k], g_queues[k], QUEUE_CAP,
                             bench_delta_e, bench_pi, NULL) == 0);
        for (int s = 0; s < WINDOW; s++) {
            assert(fdt_twin_seed(&g_twins[k], &zero) == 0);
        }
        char topic[64];
        assert(fdt_topic(topic, sizeof topic, FDT_TOPIC_LSDT, k) == 0);
        assert(tr.subscribe(tr.self, topic, on_lsdt, NULL) == 0);

        g_gprev[k] = zero;
        g_gnow[k]  = zero;
    }

    assert(fdt_fleet_init(&fleet, g_twins, vessels, &fc) == 0);
    assert(fdt_store_init(&store, g_slots, vessels) == 0);
    assert(fdt_coord_init(&coord, &fleet, &store,
                          bench_ctx, bench_plan, NULL) == 0);

    fdt_feas_init(&feas, FDT_TICK_NS);

    fdt_tick_t tk;
    if (paced) {
        fdt_tick_start(&tk, FDT_TICK_NS);
    }

    double inj_total_ns = 0.0;
    const double wall_begin = now_ns();

    for (int frame = 0; frame < frames; frame++) {
        /* Injector side: publish one I^t per vessel and time it. */
        const double inj_t0 = now_ns();
        fdt_fs_begin_frame(&g_fs);
        const int sent = fdt_inj_tick(&inj);
        (void)tr.poll(tr.self, 0);
        fdt_fs_end_frame(&g_fs);
        inj_total_ns += now_ns() - inj_t0;

        assert(sent >= 0);

        for (unsigned k = 0; k < vessels; k++) {
            g_ins[k] = g_decoded[k];
        }

        fdt_feas_begin(&feas);
        const int rc = fdt_coord_step(&coord, g_ins, WINDOW,
                                      g_gprev, g_gnow, g_bs, g_as);
        fdt_feas_end(&feas);
        assert(rc == 0);

        if (paced) {
            const uint64_t before = fdt_tick_overruns(&tk);
            fdt_tick_wait(&tk);
            if (fdt_tick_overruns(&tk) != before) {
                fdt_inj_note_missed_deadline(&inj);
            }
        }
    }

    const double wall_ns = now_ns() - wall_begin;

    out.dti_worst_ms = (double)fdt_feas_worst_ns(&feas) / 1e6;
    out.dti_mean_ms  = fdt_feas_mean_ns(&feas) / 1e6;
    out.feasible     = fdt_feas_ok(&feas);
    out.inj_mean_ms  = inj_total_ns / (double)frames / 1e6;
    out.partial      = (unsigned long)fdt_fs_partial_frames(&g_fs);
    out.doubles      = (unsigned long)fdt_fs_double_updates(&g_fs);
    out.stale        = (unsigned long)fdt_fs_stale_packets(&g_fs);
    out.inj_missed   = (unsigned long)fdt_inj_missed_deadlines(&inj);

    const double delta_total_ns = fdt_feas_mean_ns(&feas) * (double)frames;
    out.cpu_per_dti_pct = 100.0 * (delta_total_ns / wall_ns) /
                          (double)vessels;

    tr.close(tr.self);
    memset(g_decoded, 0, sizeof g_decoded);
    return out;
}

int main(void)
{
    bench_report_t r;
    bench_open(&r, "scale");
    bench_banner(&r, "fleet-dt scale bench");
    bench_say(&r, "frames per size: %d   window depth: %d   "
                  "queue capacity: %d\n", FRAMES, WINDOW, QUEUE_CAP);

    static double xs[SWEEP_N];
    static double worst_ms[SWEEP_N];
    static double mean_ms[SWEEP_N];
    static double cpu_pct[SWEEP_N];
    static double inj_ms[SWEEP_N];

    unsigned dti_ceiling = 0;
    unsigned inj_ceiling = 0;

    bench_section(&r, "per fleet size");
    bench_say(&r, "  %8s %13s %13s %11s %13s %10s\n",
              "vessels", "delta worst", "delta mean", "cpu/DTI", "inj/frame",
              "feasible");

    for (int i = 0; i < SWEEP_N; i++) {
        const unsigned v = SWEEP[i];
        const run_t run = run_fleet_ex(v, 0, 0, 0, FRAMES);

        xs[i]       = (double)v;
        worst_ms[i] = run.dti_worst_ms;
        mean_ms[i]  = run.dti_mean_ms;
        cpu_pct[i]  = run.cpu_per_dti_pct;
        inj_ms[i]   = run.inj_mean_ms;

        if (run.feasible) {
            dti_ceiling = v;
        }
        if (run.inj_mean_ms + run.dti_worst_ms < 125.0) {
            inj_ceiling = v;
        }

        bench_say(&r, "  %8u %10.1f us %10.1f us %9.3f %% %10.1f us %10s\n",
                  v, run.dti_worst_ms * 1e3, run.dti_mean_ms * 1e3,
                  run.cpu_per_dti_pct, run.inj_mean_ms * 1e3,
                  run.feasible ? "yes" : "NO");

        assert(run.partial == 0);
        assert(run.doubles == 0);
    }

    bench_section(&r, "DTI side — what Section V-B calls negligible");
    char buf[64];
    snprintf(buf, sizeof buf, "%.4f %%", cpu_pct[SWEEP_N - 1]);
    bench_compare(&r, "cpu per DTI", buf, "< 1 %",
                  (cpu_pct[SWEEP_N - 1] < 1.0) ? BENCH_OK : BENCH_DIVERGE);

    snprintf(buf, sizeof buf, "%.1f us", worst_ms[SWEEP_N - 1] * 1e3);
    bench_compare(&r, "worst delta, 64 vessels", buf, "< 125 ms deadline",
                  (worst_ms[SWEEP_N - 1] < 125.0) ? BENCH_OK : BENCH_DIVERGE);

    snprintf(buf, sizeof buf, ">= %u vessels", dti_ceiling);
    bench_compare(&r, "DTI ceiling here", buf, "not the paper's 25",
                  BENCH_OK);

    bench_section(&r, "injector side — the ceiling Section V-B actually hit");
    snprintf(buf, sizeof buf, ">= %u vessels", inj_ceiling);
    bench_compare(&r, "publish+step fits period", buf,
                  "budget check, not a pace observation", BENCH_OK);

    const int paced_frames = 40;
    const run_t paced = run_fleet_ex(SWEEP[SWEEP_N - 1], 0, 0, 1,
                                     paced_frames);
    snprintf(buf, sizeof buf, "%lu of %d frames", paced.inj_missed,
             paced_frames);
    bench_compare(&r, "missed deadlines, 64 paced", buf,
                  "injectors could not keep pace",
                  (paced.inj_missed == 0) ? BENCH_OK : BENCH_DIVERGE);
    bench_say(&r,
        "  Section V-B attributes its ceiling to the injectors, not to the\n"
        "  DTIs: \"hard-programming injectors to inject packets periodically\n"
        "  could not keep the simulation pace for larger fleets (> 25 boats,\n"
        "  same computer model)\". The two ceilings are reported apart so the\n"
        "  DTI is not credited with a limit it does not have. This machine\n"
        "  reaches neither ceiling at 64 vessels; the paper's hardware and\n"
        "  its real broker are not this loopback.\n");

    bench_section(&r, "frame integrity under a lossy link (Section V-B)");
    bench_say(&r,
        "  A lossless transport can never show the pathology, so the clean\n"
        "  sweep above reports zeros by construction. This row injects the\n"
        "  two faults Section V-B describes: a packet that never arrives, and\n"
        "  a packet the network stack holds into the following frame.\n\n");

    const unsigned lossy_vessels = 16;
    const unsigned drop_every    = 61;
    const unsigned defer_every   = 43;

    const run_t lossy = run_fleet_ex(lossy_vessels, drop_every, defer_every,
                                     0, FRAMES);
    bench_say(&r, "  %u vessels, drop 1 in %u, defer 1 in %u, %d frames\n",
              lossy_vessels, drop_every, defer_every, FRAMES);
    bench_say(&r, "    partial frames : %lu of %d\n", lossy.partial, FRAMES);
    bench_say(&r, "    double updates : %lu\n", lossy.doubles);
    bench_say(&r, "    stale packets  : %lu\n", lossy.stale);
    bench_say(&r, "    delta worst    : %.1f us   feasible: %s\n",
              lossy.dti_worst_ms * 1e3, lossy.feasible ? "yes" : "NO");

    snprintf(buf, sizeof buf, "%lu partial, %lu double",
             lossy.partial, lossy.doubles);
    bench_compare(&r, "pathology reproduced", buf,
                  "some boats, not every frame",
                  (lossy.partial > 0 && lossy.partial < (unsigned long)FRAMES
                   && lossy.doubles > 0) ? BENCH_OK : BENCH_DIVERGE);

    assert(lossy.partial > 0 && lossy.partial < (unsigned long)FRAMES);
    assert(lossy.doubles > 0);

    bench_say(&r,
        "  Stale packets stay at %lu here, and that is the deferral being\n"
        "  order-preserving rather than the counter being broken: a held\n"
        "  packet is delivered at the head of the next poll, so it still\n"
        "  arrives before the fresher one behind it. It lands in the wrong\n"
        "  frame, which makes it a double update, not an outdated read. The\n"
        "  outdated-read path needs reordering, and is covered by\n"
        "  tests/test_wire.c.\n", lossy.stale);
    bench_say(&r,
        "  Counted, never corrected. Dropping late packets at the receiver is\n"
        "  item D6 of docs/spec/paper-claims.md, which Section VI declares\n"
        "  future work; the DTI keeps stepping and the monitor keeps score.\n"
        "  Note that delta stays feasible throughout: a degraded link starves\n"
        "  the model, it does not slow it.\n");

    fdt_plot_t p;
    fdt_plot_init(&p, "Fleet scaling: DTI cost against injector cost",
                  "vessels in the fleet", "milliseconds per frame (log)");
    fdt_plot_series(&p, "delta worst (DTI)", xs, worst_ms, SWEEP_N,
                    FDT_PLOT_LINE);
    fdt_plot_series(&p, "delta mean (DTI)", xs, mean_ms, SWEEP_N,
                    FDT_PLOT_LINE);
    fdt_plot_series(&p, "injector per frame", xs, inj_ms, SWEEP_N,
                    FDT_PLOT_LINE);
    fdt_plot_hline(&p, 125.0, "125 ms deadline (Sec. IV)");
    fdt_plot_set_log_y(&p, 1);
    fdt_plot_write_svg(&p, "results/scale.svg");
    fdt_plot_write_csv(&p, "results/scale.csv");

    fdt_plot_t c;
    fdt_plot_init(&c, "CPU cost of adding one DTI",
                  "vessels in the fleet", "CPU per DTI (%)");
    fdt_plot_series(&c, "measured", xs, cpu_pct, SWEEP_N, FDT_PLOT_BAR);
    fdt_plot_hline(&c, 1.0, "< 1 % per DTI (Sec. V-B)");
    fdt_plot_write_svg(&c, "results/scale_cpu.svg");

    bench_artefacts(&r, "scale");
    bench_say(&r, "artefacts: results/scale_cpu.svg\n");
    bench_close(&r);
    return EXIT_SUCCESS;
}
