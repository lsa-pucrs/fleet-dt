/**
 * @file fdt_webots_controller.c
 * @brief The custom WeBots module that runs delta, Section IV.
 *
 * Section IV: "A custom module within WeBots implements delta in C language
 * and carries out the dynamics of the model. The simulation frequency is
 * 8 Hz, i.e. delta is a hard real-time task with deadline of 125 ms."
 *
 * Section III adds where it sits: "A 3D model derived from the DTP resides in
 * a WeBots world as a DTI inside the VE, receiving input from the MQTT
 * infrastructure (except for the camera feed, directly driven by the RTSP
 * client in MCS)."
 *
 * This file is that module. It is the one place where the model's tick and
 * the simulator's tick are the same tick: wb_robot_step() is called with the
 * model's own period, so a frame of the twin is a frame of the world. Letting
 * them drift would make the 3D view a rendering of a different instant than
 * the one the coordinator just computed, and Section I feature (iii) promises
 * a near-real-time visual reference rather than an approximate one.
 *
 * Built by `make webots`, which skips with a notice when WEBOTS_HOME is
 * unset. Nothing in the core library links the WeBots controller library.
 *
 * @note The CPU figures of Section V-A — "running WeBots adds 10% CPU usage
 *       for the first boat and less than 1% for subsequent boats" — are a
 *       boundary. They are a property of the renderer and cannot be
 *       reproduced by `make bench`; see C22 in docs/spec/paper-claims.md.
 */
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "injector/injector.h"

#include <fleet_dt/codec.h>
#include <fleet_dt/coordinator.h>
#include <fleet_dt/envelope.h>
#include <fleet_dt/feasibility.h>
#include <fleet_dt/framesync.h>
#include <fleet_dt/tick.h>
#include <fleet_dt/transport.h>
#include <fleet_dt/version.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** Vessels this world holds. Must match the number of Robot nodes. */
#define VESSELS 2

/** Queue depth per vessel: 48 * CAP bytes, the 48d bound of Section IV. */
#define QUEUE_CAP 16

/** Window depth, deeper than one so the trend term of Section V-A exists. */
#define WINDOW 4

/** Frames between status lines on the simulator console. */
#define STATUS_EVERY 40

/**
 * WeBots node DEF names, one per vessel, matching worlds/jundia_fleet.wbt.
 *
 * Resolved by DEF rather than by display name because equation (4) indexes
 * vessels, and a lookup by the string shown in the scene tree would break the
 * moment someone renames a node in the editor. tests/test_world.c checks that
 * each of these exists in the world exactly once, which is the half of this
 * file that can be verified without the SDK.
 */
static const char *const VESSEL_DEF[VESSELS] = { "PINTADO", "TILAPIA" };

static fdt_state_t     g_queues[VESSELS][QUEUE_CAP];
static fdt_twin_t      g_twins[VESSELS];
static fdt_state_t     g_slots[VESSELS];
static fdt_input_t     g_decoded[VESSELS];
static fdt_input_t     g_ins[VESSELS];
static fdt_goal_t      g_gprev[VESSELS];
static fdt_goal_t      g_gnow[VESSELS];
static fdt_state_t     g_bs[VESSELS];
static fdt_actuation_t g_as[VESSELS];
static uint32_t        g_seqs[VESSELS];
static uint8_t         g_hits[VESSELS];
static fdt_framesync_t g_fs;
static fdt_feas_t      g_feas[VESSELS];

/** Supervisor handles for writing the twin state into the 3D world. */
static WbFieldRef g_translation[VESSELS];
static WbFieldRef g_rotation[VESSELS];

/** Fleet context: yaw spread, standing in for inter-vessel distance. */
typedef struct {
    float spread_deg;
    float throttle_ceiling_pct;
} fleet_ctx_t;

/** Degrees per radian. */
#define RAD2DEG 57.2957795f

/**
 * delta^e: the dynamics Section IV says this module carries out.
 *
 * A placeholder integrator, as in examples/daemon.c. Substituting the real
 * hull model is the point of the file being here rather than in the library:
 * the library owns the frame, the application owns the physics.
 */
static void webots_delta_e(const fdt_queue_t *q, size_t n,
                           const fdt_input_t *in, const fdt_goal_t *g_prev,
                           fdt_state_t *out, void *ctx, void *fleet_ctx)
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
    out->yaw_deg     += rate * dt_s * RAD2DEG;

    if (in != NULL) {
        out->lat_deg = in->gps_lat_deg;
        out->lon_deg = in->gps_lon_deg;
        out->alt_m   = in->gps_alt_m;
    }
    if (newest != NULL && oldest != NULL && n > 1) {
        out->pitch_deg = (newest->yaw_deg - oldest->yaw_deg) /
                         (float)(n - 1);
    }
}

static void webots_pi(const fdt_state_t *b, const fdt_goal_t *g_now,
                      fdt_actuation_t *out, void *ctx, void *fleet_ctx)
{
    (void)ctx;

    const fleet_ctx_t *fc = (const fleet_ctx_t *)fleet_ctx;
    out->cage_rad     = (g_now->yaw_deg - b->yaw_deg) * 0.01f;
    out->throttle_pct = (fc != NULL) ? fc->throttle_ceiling_pct : 100.0f;
}

static void webots_ctx(const fdt_store_t *bt, void *fleet_ctx, void *user)
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

/** Goals arrive from the MCS; this stands in until that client is wired. */
static void webots_plan(const fdt_store_t *bt, const void *fleet_ctx,
                        fdt_goal_t *goals_out, size_t n, void *user)
{
    (void)bt;
    (void)fleet_ctx;
    (void)user;

    for (size_t k = 0; k < n; k++) {
        goals_out[k].yaw_deg = 45.0f + 45.0f * (float)(k % 2);
    }
}

/** Telemetry arriving from the boats over MQTT. */
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
    if (env.kind != FDT_ENV_INPUT || env.vessel >= VESSELS) {
        return;
    }

    /* Judged, never withheld: Section VI lists dropping late packets as
     * future work, so the verdict is counted and the packet still lands. */
    (void)fdt_fs_accept(&g_fs, &env);
    (void)fdt_dec_input(payload, env.payload_len, &g_decoded[env.vessel]);
}

/** Writes one twin's state into the 3D world the operator watches. */
static void render_vessel(size_t k, const fdt_state_t *b)
{
    if (g_translation[k] == NULL || g_rotation[k] == NULL) {
        return;
    }

    /* Degrees of latitude and longitude onto world metres. A local tangent
     * plane is adequate for a lagoon, and the boats of Section II never leave
     * one. */
    const double metres_per_deg = 111320.0;
    const double x = (double)b->lon_deg * metres_per_deg;
    const double z = (double)b->lat_deg * metres_per_deg;
    const double position[3] = { x, (double)b->alt_m, z };
    wb_supervisor_field_set_sf_vec3f(g_translation[k], position);

    /* Yaw about the vertical axis. Table I gives the angle in degrees and
     * WeBots wants radians, so the unit is crossed here rather than assumed. */
    const double yaw_rad = (double)b->yaw_deg / (double)RAD2DEG;
    const double rotation[4] = { 0.0, 1.0, 0.0, yaw_rad };
    wb_supervisor_field_set_sf_rotation(g_rotation[k], rotation);
}

int main(int argc, char **argv)
{
    const char *host = (argc > 1) ? argv[1] : "localhost";
    (void)host; /* Used once the MQTT transport is linked in; see below. */

    wb_robot_init();

    /* The simulator's tick is the model's tick. Not a coincidence to be
     * maintained by hand: it is read from the same constant. */
    const int step_ms = (int)(FDT_TICK_NS / 1000000L);

    for (size_t k = 0; k < VESSELS; k++) {
        WbNodeRef node = wb_supervisor_node_get_from_def(VESSEL_DEF[k]);
        if (node != NULL) {
            g_translation[k] = wb_supervisor_node_get_field(node,
                                                            "translation");
            g_rotation[k] = wb_supervisor_node_get_field(node, "rotation");
        } else {
            fprintf(stderr, "no node DEF %s in this world\n", VESSEL_DEF[k]);
        }
    }

    /* Transport. Over a real deployment this is fdt_mqtt_open(host, 1883,
     * "webots-dti", &tr) from adapters/mqtt; the loopback keeps the
     * controller runnable in a world with no broker attached. */
    static fdt_loop_t loop;
    fdt_transport_t tr = fdt_loop_transport(&loop);
    static fdt_inj_t inj;
    fdt_inj_init(&inj, &tr, VESSELS, 8.0, 20260817u);

    const fdt_state_t zero = {0};
    for (size_t k = 0; k < VESSELS; k++) {
        fdt_twin_init(&g_twins[k], g_queues[k], QUEUE_CAP,
                      webots_delta_e, webots_pi, NULL);
        for (int s = 0; s < WINDOW; s++) {
            fdt_twin_seed(&g_twins[k], &zero);
        }
        char topic[64];
        fdt_topic(topic, sizeof topic, FDT_TOPIC_LSDT, (unsigned)k);
        tr.subscribe(tr.self, topic, on_lsdt, NULL);
        fdt_feas_init(&g_feas[k], FDT_TICK_NS);
    }

    fdt_fs_init(&g_fs, g_seqs, g_hits, VESSELS);

    static fleet_ctx_t fc = { .spread_deg = 0.0f,
                              .throttle_ceiling_pct = 60.0f };
    static fdt_fleet_t fleet;
    static fdt_store_t store;
    static fdt_coord_t coord;

    fdt_fleet_init(&fleet, g_twins, VESSELS, &fc);
    fdt_store_init(&store, g_slots, VESSELS);
    fdt_coord_init(&coord, &fleet, &store, webots_ctx, webots_plan, NULL);

    /* One status line every STATUS_EVERY frames. A simulator console that
     * says nothing is indistinguishable from one whose controller died, and
     * the WeBots log reports a controller that returns before the simulation
     * ends as having "crashed" -- so silence here reads as failure. */
    size_t frames = 0;

    printf("fdt_controller: %s, %d vessels, %d ms frame, window %d\n",
           fdt_version(), VESSELS, step_ms, WINDOW);
    fflush(stdout);

    while (wb_robot_step(step_ms) != -1) {
        fdt_fs_begin_frame(&g_fs);
        fdt_inj_tick(&inj);
        tr.poll(tr.self, 0);
        fdt_fs_end_frame(&g_fs);

        for (size_t k = 0; k < VESSELS; k++) {
            g_ins[k] = g_decoded[k];
        }

        fdt_feas_begin(&g_feas[0]);
        const int rc = fdt_coord_step(&coord, g_ins, WINDOW,
                                      g_gprev, g_gnow, g_bs, g_as);
        fdt_feas_end(&g_feas[0]);

        if (rc != 0) {
            fprintf(stderr, "fleet step failed\n");
            continue;
        }

        /* Section I feature (iii): the near-real-time 3D reference. It is
         * written from the state the coordinator produced this very frame. */
        for (size_t k = 0; k < VESSELS; k++) {
            render_vessel(k, &g_bs[k]);
        }

        /* A^t travels back to the boats. Over a real link this is where the
         * latency Section V-A observes is incurred. */
        for (size_t k = 0; k < VESSELS; k++) {
            uint8_t payload[FDT_WIRE_ACT_BYTES];
            if (fdt_enc_act(&g_as[k], payload, sizeof payload) < 0) {
                continue;
            }
            const fdt_env_t env = { .magic = FDT_ENV_MAGIC,
                                    .vessel = (uint16_t)k,
                                    .kind = FDT_ENV_ACT,
                                    .flags = 0,
                                    .seq = g_seqs[k],
                                    .payload_len = FDT_WIRE_ACT_BYTES };
            uint8_t frame[FDT_ENV_HEADER_BYTES + FDT_WIRE_ACT_BYTES];
            const long n = fdt_env_encode(&env, payload, sizeof payload,
                                          frame, sizeof frame);
            if (n < 0) {
                continue;
            }
            char topic[64];
            fdt_topic(topic, sizeof topic, FDT_TOPIC_ACT, (unsigned)k);
            tr.publish(tr.self, topic, frame, (size_t)n, 1);
        }

        frames++;
        if (frames % STATUS_EVERY == 0) {
            printf("fdt_controller: frame %zu  yaw %.2f / %.2f deg  "
                   "spread %.2f  worst delta %.1f us  feasible %d  "
                   "partial %llu  double %llu\n",
                   frames,
                   (double)g_bs[0].yaw_deg, (double)g_bs[1].yaw_deg,
                   (double)fc.spread_deg,
                   (double)fdt_feas_worst_ns(&g_feas[0]) / 1e3,
                   fdt_feas_ok(&g_feas[0]),
                   (unsigned long long)fdt_fs_partial_frames(&g_fs),
                   (unsigned long long)fdt_fs_double_updates(&g_fs));
            fflush(stdout);
        }

        if (!fdt_feas_ok(&g_feas[0])) {
            /* Section IV: a DTI that misses the deadline is "merely a
             * time-bounded simulation model". Worth saying out loud. */
            fprintf(stderr, "DTI infeasible: worst delta %.3f ms\n",
                    (double)fdt_feas_worst_ns(&g_feas[0]) / 1e6);
        }
    }

    printf("fdt_controller: %zu frames, worst delta %.1f us, feasible %d\n",
           frames, (double)fdt_feas_worst_ns(&g_feas[0]) / 1e3,
           fdt_feas_ok(&g_feas[0]));
    fflush(stdout);

    wb_robot_cleanup();
    return EXIT_SUCCESS;
}
