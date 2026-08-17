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
 *       for the first boat and less than 1% for subsequent boats" — are
 *       measured by tools/bench/bench_webots_cpu.c, which opens the
 *       zero-, one- and two-vessel worlds in turn and subtracts. The vessel
 *       count is discovered from the world rather than compiled in, which is
 *       what lets one binary serve all three.
 */
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "injector/injector.h"

#include <fleet_dt/codec.h>
#include <fleet_dt/coordinator.h>
#include <fleet_dt/envelope.h>
#include <fleet_dt/feasibility.h>
#include <fleet_dt/framesync.h>
#include <fleet_dt/geo.h>
#include <fleet_dt/tick.h>
#include <fleet_dt/transport.h>
#include <fleet_dt/version.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/**
 * Upper bound on vessels. The actual count is discovered at startup by asking
 * the world which DEFs resolve, so one controller binary serves a one-boat
 * world and a two-boat world without a rebuild. That is what makes the CPU
 * comparison of Section V-A -- the first boat against each subsequent one --
 * a matter of opening a different world rather than of editing this file.
 */
#define MAX_VESSELS 8

/** Queue depth per vessel: 48 * CAP bytes, the 48d bound of Section IV. */
#define QUEUE_CAP 16

/** Window depth, deeper than one so the trend term of Section V-A exists. */
#define WINDOW 4

/** Frames between status lines on the simulator console. */
#define STATUS_EVERY 40

/**
 * Frame at which FDT_SHOT, if set, captures the 3D view to that path.
 *
 * Numbers cannot tell you the view is right. Both position bugs this
 * controller has had -- hulls rendered 5700 km away, then hulls climbing out
 * of the water -- left every counter green and were caught by a person
 * looking at the viewport. This exists so that looking does not require a
 * person to be present.
 */
#define SHOT_FRAME 80  /* FDT_SHOT_FRAME overrides */

/**
 * WeBots node DEF names, one per vessel, matching worlds/jundia_fleet.wbt.
 *
 * Resolved by DEF rather than by display name because equation (4) indexes
 * vessels, and a lookup by the string shown in the scene tree would break the
 * moment someone renames a node in the editor. tests/test_world.c checks that
 * each of these exists in the world exactly once, which is the half of this
 * file that can be verified without the SDK.
 */
static const char *const VESSEL_DEF[MAX_VESSELS] = {
    "PINTADO", "TILAPIA", "DOURADO", "PIAVA",
    "TRAIRA",  "LAMBARI", "CARPA",   "PACU",
};

/** Vessels this world actually holds; resolved in main(). */
static size_t g_vessels;

/** Frame at which the view is captured; FDT_SHOT_FRAME overrides. */
static size_t g_shot_frame = SHOT_FRAME;

static fdt_state_t     g_queues[MAX_VESSELS][QUEUE_CAP];
static fdt_twin_t      g_twins[MAX_VESSELS];
static fdt_state_t     g_slots[MAX_VESSELS];
static fdt_input_t     g_decoded[MAX_VESSELS];
static fdt_input_t     g_ins[MAX_VESSELS];
static fdt_goal_t      g_gprev[MAX_VESSELS];
static fdt_goal_t      g_gnow[MAX_VESSELS];
static fdt_state_t     g_bs[MAX_VESSELS];
static fdt_actuation_t g_as[MAX_VESSELS];
static uint32_t        g_seqs[MAX_VESSELS];
static uint8_t         g_hits[MAX_VESSELS];
static fdt_framesync_t g_fs;
static fdt_feas_t      g_feas[MAX_VESSELS];

/** Supervisor handles for writing the twin state into the 3D world. */
static WbNodeRef  g_node[MAX_VESSELS];
static WbFieldRef g_translation[MAX_VESSELS];
static WbFieldRef g_rotation[MAX_VESSELS];

/**
 * Where the world put each hull, and the geodetic point taken to match it.
 *
 * The twin reports a latitude and a longitude; the world wants metres from an
 * arbitrary origin. The two are reconciled by anchoring rather than by
 * converting, for the reason spelled out in render_vessel().
 */
static double g_origin[MAX_VESSELS][3];
static double g_ref_lat[MAX_VESSELS];
static double g_ref_lon[MAX_VESSELS];
static int    g_anchored[MAX_VESSELS];

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
    if (env.kind != FDT_ENV_INPUT || (size_t)env.vessel >= g_vessels) {
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
    /* FDT_NO_POSE leaves the hulls wherever the world put them. It exists
     * because "the boats are not in the view" has two causes that look
     * identical -- the twin writing a wrong pose, or the camera pointed
     * somewhere else -- and this separates them in one run. */
    if (getenv("FDT_NO_POSE") != NULL) {
        return;
    }

    /* A state produced before any telemetry arrived carries no position.
     * Rendering the zero it holds would drag the hull to the null island. */
    if (b->lat_deg == 0.0f && b->lon_deg == 0.0f) {
        return;
    }

    /* Anchor on the first real fix: that geodetic point is taken to be where
     * the world already placed this hull.
     *
     * Converting latitude and longitude to world metres directly does not
     * work. Multiplying -51.17 degrees by metres per degree puts the vessel
     * five thousand kilometres from the origin, and the water in this world is
     * a thousand metres square, so the hulls leave the scene on the first
     * frame and the operator sees an empty lagoon. */
    if (!g_anchored[k]) {
        g_ref_lat[k]  = (double)b->lat_deg;
        g_ref_lon[k]  = (double)b->lon_deg;
        g_anchored[k] = 1;
    }

    /* Displacement on a local tangent plane, which is all the vessels of
     * Section II ever cross. The arithmetic lives in the library so that
     * tests/test_geo.c can hold it to account; doing it inline here is how it
     * was wrong in the first place. */
    double dx = 0.0;
    double dz = 0.0;
    fdt_geo_offset(g_ref_lat[k], g_ref_lon[k],
                   (double)b->lat_deg, (double)b->lon_deg, &dx, &dz);

    /* WeBots R2025a is ENU: index 0 is east, index 1 is north, index 2 is up.
     *
     * Not the Y-up convention most 3D tooling uses, and assuming that one adds
     * the northward displacement to the altitude instead: the hulls climb out
     * of the lagoon and fly. The world states the convention plainly if you
     * read it -- the hulls sit at z = 0.24, a hand's breadth above water whose
     * box is 0.7 thick, and their rotation axis is 0 0 1.
     *
     * The height is left where the world put it. Altitude belongs to buoyancy,
     * which is the fluid node's business; a twin that wrote its own altitude
     * every frame would be overriding the physics it is supposed to mirror. */
    const double position[3] = { g_origin[k][0] + dx,   /* east */
                                 g_origin[k][1] + dz,   /* north */
                                 g_origin[k][2] };      /* up, untouched */
    wb_supervisor_field_set_sf_vec3f(g_translation[k], position);

    /* Yaw turns about the vertical axis, which in ENU is Z. Table I gives the
     * angle in degrees and WeBots wants radians, so the unit is crossed here
     * rather than assumed. */
    const double yaw_rad = (double)b->yaw_deg / (double)RAD2DEG;
    const double rotation[4] = { 0.0, 0.0, 1.0, yaw_rad };
    wb_supervisor_field_set_sf_rotation(g_rotation[k], rotation);

    /* A solid with Physics does not simply appear where its translation field
     * says. ODE keeps its own body state, and moving the field without telling
     * it leaves the two disagreeing: the engine sees a body that teleported,
     * infers an enormous velocity, and throws the hull away from the fleet.
     * Resetting the physics re-seats the body at the new pose with its
     * velocities zeroed, which is what a twin wants -- the pose comes from the
     * telemetry, not from the engine integrating its own guess. */
    if (g_node[k] != NULL) {
        wb_supervisor_node_reset_physics(g_node[k]);
    }
}

int main(int argc, char **argv)
{
    const char *host = (argc > 1) ? argv[1] : "localhost";
    (void)host; /* Used once the MQTT transport is linked in; see below. */

    wb_robot_init();

    /* The simulator's tick is the model's tick. Not a coincidence to be
     * maintained by hand: it is read from the same constant. */
    const int step_ms = (int)(FDT_TICK_NS / 1000000L);

    /* Ask the world how many vessels it holds. The DEFs are tried in order and
     * the first miss ends the fleet, so a world with PINTADO alone yields one
     * vessel and the same binary drives it. */
    const char *shot_at = getenv("FDT_SHOT_FRAME");
    if (shot_at != NULL && atoi(shot_at) > 0) {
        g_shot_frame = (size_t)atoi(shot_at);
    }

    g_vessels = 0;
    for (size_t k = 0; k < MAX_VESSELS; k++) {
        WbNodeRef node = wb_supervisor_node_get_from_def(VESSEL_DEF[k]);
        if (node == NULL) {
            break;
        }
        g_node[k]        = node;
        g_translation[k] = wb_supervisor_node_get_field(node, "translation");
        g_rotation[k]    = wb_supervisor_node_get_field(node, "rotation");

        /* Where the world placed this hull; the twin's motion is rendered as
         * a displacement from here rather than as an absolute position. */
        const double *t = wb_supervisor_field_get_sf_vec3f(g_translation[k]);
        if (t != NULL) {
            g_origin[k][0] = t[0];
            g_origin[k][1] = t[1];
            g_origin[k][2] = t[2];
        }
        g_anchored[k] = 0;
        g_vessels++;
    }

    if (g_vessels == 0) {
        fprintf(stderr, "no vessel DEF found; this world holds no DTI\n");
        wb_robot_cleanup();
        return EXIT_FAILURE;
    }

    /* Transport. Over a real deployment this is fdt_mqtt_open(host, 1883,
     * "webots-dti", &tr) from adapters/mqtt; the loopback keeps the
     * controller runnable in a world with no broker attached. */
    static fdt_loop_t loop;
    fdt_transport_t tr = fdt_loop_transport(&loop);
    static fdt_inj_t inj;
    fdt_inj_init(&inj, &tr, (unsigned)g_vessels, 8.0, 20260817u);

    const fdt_state_t zero = {0};
    for (size_t k = 0; k < g_vessels; k++) {
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

    fdt_fs_init(&g_fs, g_seqs, g_hits, g_vessels);

    static fleet_ctx_t fc = { .spread_deg = 0.0f,
                              .throttle_ceiling_pct = 60.0f };
    static fdt_fleet_t fleet;
    static fdt_store_t store;
    static fdt_coord_t coord;

    fdt_fleet_init(&fleet, g_twins, g_vessels, &fc);
    fdt_store_init(&store, g_slots, g_vessels);
    fdt_coord_init(&coord, &fleet, &store, webots_ctx, webots_plan, NULL);

    /* One status line every STATUS_EVERY frames. A simulator console that
     * says nothing is indistinguishable from one whose controller died, and
     * the WeBots log reports a controller that returns before the simulation
     * ends as having "crashed" -- so silence here reads as failure. */
    size_t frames = 0;

    printf("fdt_controller: %s, %zu vessels, %d ms frame, window %d\n",
           fdt_version(), g_vessels, step_ms, WINDOW);
    fflush(stdout);

    while (wb_robot_step(step_ms) != -1) {
        fdt_fs_begin_frame(&g_fs);
        fdt_inj_tick(&inj);
        tr.poll(tr.self, 0);
        fdt_fs_end_frame(&g_fs);

        for (size_t k = 0; k < g_vessels; k++) {
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
        for (size_t k = 0; k < g_vessels; k++) {
            render_vessel(k, &g_bs[k]);
        }

        /* A^t travels back to the boats. Over a real link this is where the
         * latency Section V-A observes is incurred. */
        for (size_t k = 0; k < g_vessels; k++) {
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

        if (frames == g_shot_frame) {
            const char *shot = getenv("FDT_SHOT");
            if (shot != NULL && shot[0] != '\0') {
                wb_supervisor_export_image(shot, 100);
                printf("fdt_controller: view captured to %s\n", shot);
                fflush(stdout);
            }
        }

        if (frames % STATUS_EVERY == 0) {
            /* All three axes, named. The earlier line printed indices 0 and 2
             * and called them a position, so a hull climbing out of the water
             * read as ordinary motion across it. */
            const double *pos0 = (g_translation[0] != NULL)
                ? wb_supervisor_field_get_sf_vec3f(g_translation[0]) : NULL;
            printf("fdt_controller: frame %zu  yaw %.2f / %.2f deg  "
                   "spread %.2f  worst delta %.1f us  feasible %d  "
                   "partial %llu  double %llu  pos0 E%.1f N%.1f U%.2f\n",
                   frames,
                   (double)g_bs[0].yaw_deg,
                   (double)g_bs[g_vessels > 1 ? 1 : 0].yaw_deg,
                   (double)fc.spread_deg,
                   (double)fdt_feas_worst_ns(&g_feas[0]) / 1e3,
                   fdt_feas_ok(&g_feas[0]),
                   (unsigned long long)fdt_fs_partial_frames(&g_fs),
                   (unsigned long long)fdt_fs_double_updates(&g_fs),
                   pos0 != NULL ? pos0[0] : 0.0,
                   pos0 != NULL ? pos0[1] : 0.0,
                   pos0 != NULL ? pos0[2] : 0.0);
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
