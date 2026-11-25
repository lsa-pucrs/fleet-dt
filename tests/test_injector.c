/**
 * @file test_injector.c
 * @brief The synthetic telemetry injectors of Section V-B, publishing
 *        through the regulated client of Section III.
 */
#include "injector/injector.h"

#include <fleet_dt/assembler.h>
#include <fleet_dt/codec.h>
#include <fleet_dt/envelope.h>

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Injectors and loops are large; keeping them static keeps the stack small. */
static fdt_loop_t loop_a;
static fdt_loop_t loop_b;
static fdt_loop_t loop_c;
static fdt_inj_t  inj_a;
static fdt_inj_t  inj_b;
static fdt_inj_t  inj_c;

/** Counts arriving messages per sensor kind. */
typedef struct {
    unsigned msgs;
    unsigned per_sensor[FDT_SENSOR_COUNT];
    unsigned foreign;
    uint32_t last_seq;
} sink_t;

static void on_msg(const char *topic, const uint8_t *buf, size_t len,
                   void *user)
{
    sink_t *s = (sink_t *)user;
    s->msgs++;

    fdt_env_t env;
    const uint8_t *payload = NULL;
    if (fdt_env_decode(buf, len, &env, &payload) < 0) {
        s->foreign++;
        return;
    }
    const int k = fdt_env_kind_sensor(env.kind);
    if (k < 0 || payload == NULL) {
        s->foreign++;
        return;
    }

    char expect[64];
    assert(fdt_sensor_topic(expect, sizeof expect, env.vessel,
                            FDT_TOPIC_LSDT,
                            fdt_sensor_name((fdt_sensor_t)k)) == 0);
    assert(strcmp(expect, topic) == 0);
    assert(env.payload_len == fdt_sensor_bytes((fdt_sensor_t)k));

    s->per_sensor[k]++;
    s->last_seq = env.seq;
}

static void test_init_guards(void)
{
    fdt_transport_t tr = fdt_loop_transport(&loop_a);

    assert(fdt_inj_init(&inj_a, &tr, 4, 8.0, 12345) == 0);
    assert(fdt_inj_init(&inj_a, NULL, 4, 8.0, 1) == -1);
    assert(fdt_inj_init(&inj_a, &tr, 0, 8.0, 1) == -1);
    assert(fdt_inj_init(&inj_a, &tr, FDT_INJ_MAX_VESSELS + 1, 8.0, 1) == -1);
    assert(fdt_inj_init(&inj_a, &tr, 4, 0.0, 1) == -1);
    assert(fdt_inj_init(NULL, &tr, 4, 8.0, 1) == -1);

    assert(fdt_inj_init(&inj_a, &tr, 4, 8.0, 1) == 0);
    const double bad[FDT_SENSOR_COUNT] = { 1000.0, 0.0, 50.0, 10.0 };
    assert(fdt_inj_configure(&inj_a, bad, 0) == -1);
    assert(fdt_inj_configure(NULL, NULL, 0) == -1);

    assert(fdt_inj_published(NULL) == 0);
    assert(fdt_inj_tick(NULL) == -1);
    assert(fdt_inj_client(&inj_a, 4) == NULL);
}

/**
 * Slide 8 and 13: one message per sensor per tick, four per boat, each on
 * its own topic, and the four of a tick share its sequence number.
 */
static void test_one_message_per_sensor_per_tick(void)
{
    fdt_transport_t tr = fdt_loop_transport(&loop_a);
    sink_t sink = {0};

    assert(fdt_inj_init(&inj_a, &tr, 4, 8.0, 12345) == 0);
    assert(tr.subscribe(tr.self, FDT_TOPIC_LSDT_ALL, on_msg, &sink) == 0);

    assert(fdt_inj_tick(&inj_a) == 16);
    assert(tr.poll(tr.self, 0) == 16);
    assert(sink.msgs == 16 && sink.foreign == 0);
    for (int s = 0; s < FDT_SENSOR_COUNT; s++) {
        assert(sink.per_sensor[s] == 4);
    }
    assert(fdt_inj_published(&inj_a) == 16);

    const uint32_t seq_first = sink.last_seq;
    assert(fdt_inj_tick(&inj_a) == 16);
    assert(tr.poll(tr.self, 0) == 16);
    assert(sink.last_seq == seq_first + 1);

    /* The IMU took 250 samples in two ticks; two reached the link. */
    const fdt_reg_t *imu = fdt_regpub_reg(fdt_inj_client(&inj_a, 0),
                                          FDT_SENSOR_IMU);
    assert(fdt_reg_sampled(imu) == 250);
    assert(fdt_reg_published(imu) == 2);
}

/**
 * Slide 13: one boat sends 1070 messages per second raw (1000 + 50 + 10 +
 * 10) and 32 regulated (4 sensors x 8 Hz).
 */
static void test_raw_against_regulated_rates(void)
{
    fdt_transport_t ta = fdt_loop_transport(&loop_a);
    fdt_transport_t tb = fdt_loop_transport(&loop_b);
    sink_t raw_sink = {0};
    sink_t reg_sink = {0};

    assert(fdt_inj_init(&inj_a, &ta, 1, 8.0, 9) == 0);
    assert(fdt_inj_configure(&inj_a, NULL, 1) == 0);
    assert(fdt_inj_init(&inj_b, &tb, 1, 8.0, 9) == 0);
    assert(ta.subscribe(ta.self, FDT_TOPIC_LSDT_ALL, on_msg, &raw_sink) == 0);
    assert(tb.subscribe(tb.self, FDT_TOPIC_LSDT_ALL, on_msg, &reg_sink) == 0);

    for (int k = 0; k < 8; k++) { /* one second */
        (void)fdt_inj_tick(&inj_a);
        (void)fdt_inj_tick(&inj_b);
        (void)ta.poll(ta.self, 0);
        (void)tb.poll(tb.self, 0);
    }

    assert(raw_sink.msgs == 1070);
    assert(raw_sink.per_sensor[FDT_SENSOR_IMU] == 1000);
    assert(raw_sink.per_sensor[FDT_SENSOR_BARO] == 50);
    assert(raw_sink.per_sensor[FDT_SENSOR_GPS] == 10);
    assert(raw_sink.per_sensor[FDT_SENSOR_POWER] == 10);
    assert(reg_sink.msgs == 32);
    assert(fdt_inj_published(&inj_a) == 1070);
    assert(fdt_inj_published(&inj_b) == 32);

    printf("one boat, one second: %u messages raw, %u regulated\n",
           raw_sink.msgs, reg_sink.msgs);
}

/** The twin side rebuilds I^t from the four messages of a tick. */
static void test_twin_side_assembles_input(void)
{
    fdt_transport_t tr = fdt_loop_transport(&loop_a);
    fdt_rx_slot_t   slots[2];
    uint32_t        seqs[2];
    uint8_t         hits[2];
    fdt_framesync_t fs;
    fdt_rx_t        rx;

    assert(fdt_inj_init(&inj_a, &tr, 2, 8.0, 5) == 0);
    assert(fdt_fs_init(&fs, seqs, hits, 2) == 0);
    assert(fdt_rx_init(&rx, slots, 2, &fs) == 0);

    /* The newest sample of each sensor in frame 0, as the boat holds it. */
    fdt_input_t want;
    fdt_input_t part;
    fdt_inj_sample_at(&inj_a, 1, 0.125, &want);          /* IMU j = 125 */
    fdt_inj_sample_at(&inj_a, 1, 0.1, &part);            /* GPS j = 1   */
    want.gps_lat_deg = part.gps_lat_deg;
    want.gps_lon_deg = part.gps_lon_deg;
    want.gps_alt_m   = part.gps_alt_m;
    want.vn_mps = part.vn_mps;
    want.ve_mps = part.ve_mps;
    want.vd_mps = part.vd_mps;
    fdt_inj_sample_at(&inj_a, 1, 0.12, &part);           /* baro j = 6  */
    want.press_pa = part.press_pa;
    want.temp_c   = part.temp_c;
    fdt_inj_sample_at(&inj_a, 1, 0.1, &part);            /* power j = 1 */
    want.vbat_v = part.vbat_v;
    want.ibat_a = part.ibat_a;

    fdt_rx_begin_frame(&rx);
    assert(fdt_inj_tick(&inj_a) == 8);
    for (size_t i = 0; i < loop_a.qlen; i++) {
        assert(fdt_rx_accept(&rx, loop_a.q[i].payload, loop_a.q[i].len) ==
               FDT_FS_FRESH);
    }
    loop_a.qlen = 0;
    fdt_rx_end_frame(&rx);

    const fdt_input_t *got = fdt_rx_input(&rx, 1);
    assert(got->wz_rps == want.wz_rps);
    assert(got->gps_lat_deg == want.gps_lat_deg);
    assert(got->press_pa == want.press_pa);
    assert(got->vbat_v == want.vbat_v);
    assert(got->x_left == NULL); /* the views travel off MQTT */

    assert(fdt_rx_accepted(&rx) == 8);
    assert(fdt_rx_incomplete(&rx) == 0);
    assert(fdt_fs_accepted(&fs) == 2);   /* one boat-frame per vessel */
    assert(fdt_fs_partial_frames(&fs) == 0);
}

/** Telemetry sits in the ranges Section II describes for the Jundia boat. */
static void test_plausible_telemetry(void)
{
    fdt_transport_t tr = fdt_loop_transport(&loop_a);
    fdt_input_t in;

    assert(fdt_inj_init(&inj_a, &tr, 2, 8.0, 42) == 0);
    fdt_inj_sample(&inj_a, 0, &in);

    /* An 18.5 V, 10 Ah LiPo under load. */
    assert(in.vbat_v > 17.5f && in.vbat_v < 19.5f);
    assert(in.ibat_a > 0.0f && in.ibat_a < 20.0f);

    /* Sea-level pressure, and a boat that is not upside down. */
    assert(in.press_pa > 100000.0f && in.press_pa < 103000.0f);
    assert(in.az_mps2 < -9.0f && in.az_mps2 > -10.5f);

    /* The southern Brazilian lagoons the eDNA campaign samples. */
    assert(in.gps_lat_deg < -29.0f && in.gps_lat_deg > -31.0f);
    assert(in.gps_lon_deg < -50.0f && in.gps_lon_deg > -52.0f);

    /* Views exist but never travel this path. */
    assert(in.x_left == FDT_VIEW_PRESENT);

    /* An out-of-range vessel yields a zeroed sample rather than garbage. */
    fdt_input_t none;
    fdt_inj_sample(&inj_a, 99, &none);
    assert(none.vbat_v == 0.0f);
}

/** Same seed, same telemetry: a campaign has to be repeatable. */
static void test_deterministic(void)
{
    fdt_transport_t ta = fdt_loop_transport(&loop_a);
    fdt_transport_t tb = fdt_loop_transport(&loop_b);

    assert(fdt_inj_init(&inj_a, &ta, 2, 8.0, 777) == 0);
    assert(fdt_inj_init(&inj_b, &tb, 2, 8.0, 777) == 0);

    for (int t = 0; t < 5; t++) {
        for (unsigned v = 0; v < 2; v++) {
            fdt_input_t ia;
            fdt_input_t ib;
            fdt_inj_sample(&inj_a, v, &ia);
            fdt_inj_sample(&inj_b, v, &ib);
            assert(memcmp(&ia, &ib, sizeof ia) == 0);
        }
        assert(fdt_inj_tick(&inj_a) == 8);
        assert(fdt_inj_tick(&inj_b) == 8);
        assert(loop_a.qlen == loop_b.qlen);
        for (size_t i = 0; i < loop_a.qlen; i++) {
            assert(loop_a.q[i].len == loop_b.q[i].len);
            assert(memcmp(loop_a.q[i].payload, loop_b.q[i].payload,
                          loop_a.q[i].len) == 0);
        }
        loop_a.qlen = 0;
        loop_b.qlen = 0;
    }

    /* Different seeds diverge, or the seed would be decorative. */
    fdt_transport_t tc = fdt_loop_transport(&loop_c);
    assert(fdt_inj_init(&inj_c, &tc, 2, 8.0, 778) == 0);

    fdt_input_t ia;
    fdt_input_t ic;
    fdt_inj_sample(&inj_a, 0, &ia);
    fdt_inj_sample(&inj_c, 0, &ic);
    assert(memcmp(&ia, &ic, sizeof ia) != 0);
}

/** A refused publish is counted, never retried. */
static void test_refusal_is_measured(void)
{
    fdt_transport_t tr = fdt_loop_transport(&loop_a);

    /* 257 vessels x 4 sensors = 1028 messages into a 1024-entry queue. */
    const unsigned vessels = FDT_LOOP_QUEUE / FDT_SENSOR_COUNT + 1;
    assert(fdt_inj_init(&inj_a, &tr, vessels, 8.0, 1) == 0);

    const int sent = fdt_inj_tick(&inj_a);
    assert(sent == FDT_LOOP_QUEUE);
    assert(fdt_inj_refused(&inj_a) == FDT_SENSOR_COUNT);

    fdt_inj_note_missed_deadline(&inj_a);
    assert(fdt_inj_missed_deadlines(&inj_a) == 1);
}

int main(void)
{
    test_init_guards();
    test_one_message_per_sensor_per_tick();
    test_raw_against_regulated_rates();
    test_twin_side_assembles_input();
    test_plausible_telemetry();
    test_deterministic();
    test_refusal_is_measured();

    printf("test_injector: ok\n");
    return 0;
}
