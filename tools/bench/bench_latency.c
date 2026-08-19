/**
 * @file bench_latency.c
 * @brief Two figures Section V-A keeps apart: delta time and actuation delay.
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

#define VESSELS 4
#define CAP     8
#define WINDOW  2
#define FRAMES  400

static fdt_state_t     g_queues[VESSELS][CAP];
static fdt_twin_t      g_twins[VESSELS];
static fdt_state_t     g_slots[VESSELS];
static fdt_input_t     g_ins[VESSELS];
static fdt_goal_t      g_gprev[VESSELS];
static fdt_goal_t      g_gnow[VESSELS];
static fdt_state_t     g_bs[VESSELS];
static fdt_actuation_t g_as[VESSELS];
static fdt_input_t     g_decoded[VESSELS];

/** Set when the actuation for the current frame comes back off the wire. */
static int g_act_seen;

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

/** The boat's end of the loop: the actuation has come back. */
static void on_act(const char *topic, const uint8_t *buf, size_t len,
                   void *user)
{
    (void)topic;
    (void)user;

    fdt_env_t env;
    const uint8_t *payload = NULL;
    if (fdt_env_decode(buf, len, &env, &payload) < 0 || payload == NULL) {
        return;
    }

    fdt_actuation_t a;
    if (env.kind == FDT_ENV_ACT &&
        fdt_dec_act(payload, env.payload_len, &a) > 0) {
        g_act_seen = 1;
    }
}

/** Ascending-order comparison for the percentile helper. */
static int cmp_double(const void *a, const void *b)
{
    const double x = *(const double *)a;
    const double y = *(const double *)b;
    return (x < y) ? -1 : (x > y) ? 1 : 0;
}

/** Percentile of a sorted array. */
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
    bench_open(&r, "latency");
    bench_banner(&r, "fleet-dt latency bench");
    bench_say(&r, "transport: in-process loopback   vessels: %d   "
                  "frames: %d\n", VESSELS, FRAMES);

    fdt_loop_t loop;
    fdt_transport_t tr = fdt_loop_transport(&loop);
    fdt_inj_t inj;
    fdt_fleet_t fleet;
    fdt_store_t store;
    fdt_coord_t coord;
    fdt_feas_t feas;
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
        assert(fdt_topic(topic, sizeof topic, FDT_TOPIC_ACT, k) == 0);
        assert(tr.subscribe(tr.self, topic, on_act, NULL) == 0);
    }

    assert(fdt_fleet_init(&fleet, g_twins, VESSELS, NULL) == 0);
    assert(fdt_store_init(&store, g_slots, VESSELS) == 0);
    assert(fdt_coord_init(&coord, &fleet, &store, ctx_fn, plan_fn, NULL) == 0);
    fdt_feas_init(&feas, FDT_TICK_NS);

    static double xs[FRAMES];
    static double delta_us[FRAMES];
    static double rtt_us[FRAMES];

    for (int frame = 0; frame < FRAMES; frame++) {
        /* The round trip starts when the boat publishes its telemetry. */
        const double t_publish = now_ns();
        assert(fdt_inj_tick(&inj) == VESSELS);
        (void)tr.poll(tr.self, 0);

        for (unsigned k = 0; k < VESSELS; k++) {
            g_ins[k] = g_decoded[k];
        }

        /* Measurement one: how long delta takes. Section IV's predicate. */
        fdt_feas_begin(&feas);
        const int rc = fdt_coord_step(&coord, g_ins, WINDOW,
                                      g_gprev, g_gnow, g_bs, g_as);
        const long d_ns = fdt_feas_end(&feas);
        assert(rc == 0);

        /* The actuation travels back to the boat. */
        g_act_seen = 0;
        for (unsigned k = 0; k < VESSELS; k++) {
            uint8_t payload[FDT_WIRE_ACT_BYTES];
            assert(fdt_enc_act(&g_as[k], payload, sizeof payload) > 0);

            const fdt_env_t env = { .magic = FDT_ENV_MAGIC,
                                    .vessel = (uint16_t)k,
                                    .kind = FDT_ENV_ACT,
                                    .flags = 0,
                                    .seq = (uint32_t)frame + 1u,
                                    .payload_len = FDT_WIRE_ACT_BYTES };
            uint8_t out[FDT_ENV_HEADER_BYTES + FDT_WIRE_ACT_BYTES];
            const long n = fdt_env_encode(&env, payload, sizeof payload,
                                          out, sizeof out);
            assert(n > 0);

            char topic[64];
            assert(fdt_topic(topic, sizeof topic, FDT_TOPIC_ACT, k) == 0);
            (void)tr.publish(tr.self, topic, out, (size_t)n, 1);
        }
        (void)tr.poll(tr.self, 0);

        const double t_delivered = now_ns();
        assert(g_act_seen == 1);

        xs[frame]       = (double)frame;
        delta_us[frame] = (double)d_ns / 1e3;
        rtt_us[frame]   = (t_delivered - t_publish) / 1e3;
    }

    static double sorted_delta[FRAMES];
    static double sorted_rtt[FRAMES];
    memcpy(sorted_delta, delta_us, sizeof sorted_delta);
    memcpy(sorted_rtt, rtt_us, sizeof sorted_rtt);
    qsort(sorted_delta, FRAMES, sizeof(double), cmp_double);
    qsort(sorted_rtt, FRAMES, sizeof(double), cmp_double);

    char buf[64];

    bench_section(&r, "measurement 1, delta compute time (Section IV)");
    bench_say(&r, "  p50 %8.2f us   p95 %8.2f us   p99 %8.2f us   "
                  "worst %8.2f us\n",
              pct(sorted_delta, FRAMES, 0.50), pct(sorted_delta, FRAMES, 0.95),
              pct(sorted_delta, FRAMES, 0.99), sorted_delta[FRAMES - 1]);
    snprintf(buf, sizeof buf, "%.2f us worst", sorted_delta[FRAMES - 1]);
    bench_compare(&r, "delta under the deadline", buf,
                  "delta in less than 125 ms is feasible",
                  fdt_feas_ok(&feas) ? BENCH_MATCH : BENCH_HOST);

    bench_section(&r, "measurement 2, actuation round trip (Section V-A)");
    bench_say(&r, "  I^t published -> A^t delivered\n");
    bench_say(&r, "  p50 %8.2f us   p95 %8.2f us   p99 %8.2f us   "
                  "worst %8.2f us\n",
              pct(sorted_rtt, FRAMES, 0.50), pct(sorted_rtt, FRAMES, 0.95),
              pct(sorted_rtt, FRAMES, 0.99), sorted_rtt[FRAMES - 1]);
    bench_compare(&r, "round trip over a network", "not measured here",
                  "actuation is delivered late", BENCH_EXTERNAL);
    bench_say(&r,
        "  This run uses the in-process loopback, so the round trip carries\n"
        "  no network. Section V-A's observation is about a real link, and\n"
        "  reproducing it needs the MQTT transport against a broker. What\n"
        "  this block does establish is that the two quantities are measured\n"
        "  separately: delta being feasible says nothing about when the\n"
        "  actuation arrives.\n");

    bench_section(&r, "why they are not one number");
    bench_say(&r, "  delta worst    : %8.2f us  (compute)\n",
              sorted_delta[FRAMES - 1]);
    bench_say(&r, "  round trip p50 : %8.2f us  (compute + transport)\n",
              pct(sorted_rtt, FRAMES, 0.50));
    bench_say(&r, "  The second contains the first. Adding them would count\n"
                  "  the compute twice.\n");

    fdt_plot_t p;
    fdt_plot_init(&p, "Delta compute time and actuation round trip",
                  "frame", "microseconds (log)");
    fdt_plot_series(&p, "delta compute", xs, delta_us, FRAMES, FDT_PLOT_LINE);
    fdt_plot_series(&p, "I^t -> A^t round trip", xs, rtt_us, FRAMES,
                    FDT_PLOT_LINE);
    fdt_plot_hline(&p, 125000.0, "125 ms deadline (Sec. IV)");
    fdt_plot_set_log_y(&p, 1);
    fdt_plot_write_svg(&p, "results/latency.svg");
    fdt_plot_write_csv(&p, "results/latency.csv");

    bench_artefacts(&r, "latency");
    bench_close(&r);
    tr.close(tr.self);
    return EXIT_SUCCESS;
}
