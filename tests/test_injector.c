/**
 * @file test_injector.c
 * @brief The synthetic telemetry injectors of Section V-B.
 *
 * Claim covered in part: C26. The injector's own ceiling is measured by
 * tools/bench/bench_scale.c; what is checked here is that its output is
 * well-formed and repeatable, without which a scaling result could not be
 * compared against a previous one.
 */
#include "injector/injector.h"

#include <fleet_dt/codec.h>
#include <fleet_dt/envelope.h>

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/** Decodes each arriving frame and records what it saw. */
typedef struct {
    unsigned  msgs;
    unsigned  decoded;
    uint16_t  last_vessel;
    uint32_t  last_seq;
    fdt_input_t last_input;
} sink_t;

static void on_msg(const char *topic, const uint8_t *buf, size_t len,
                   void *user)
{
    (void)topic;

    sink_t *s = (sink_t *)user;
    s->msgs++;

    fdt_env_t env;
    const uint8_t *payload = NULL;
    if (fdt_env_decode(buf, len, &env, &payload) < 0) {
        return;
    }
    if (env.kind != FDT_ENV_INPUT || payload == NULL) {
        return;
    }
    if (fdt_dec_input(payload, env.payload_len, &s->last_input) < 0) {
        return;
    }

    s->decoded++;
    s->last_vessel = env.vessel;
    s->last_seq    = env.seq;
}

static void test_init_guards(void)
{
    fdt_loop_t loop;
    fdt_transport_t tr = fdt_loop_transport(&loop);
    fdt_inj_t inj;

    assert(fdt_inj_init(&inj, &tr, 4, 8.0, 12345) == 0);
    assert(fdt_inj_init(&inj, NULL, 4, 8.0, 1) == -1);
    assert(fdt_inj_init(&inj, &tr, 0, 8.0, 1) == -1);
    assert(fdt_inj_init(&inj, &tr, 4, 0.0, 1) == -1);
    assert(fdt_inj_init(NULL, &tr, 4, 8.0, 1) == -1);

    assert(fdt_inj_published(NULL) == 0);
    assert(fdt_inj_tick(NULL) == -1);
}

/** One I^t per vessel per tick, each decodable at the far end. */
static void test_publishes_one_per_vessel(void)
{
    fdt_loop_t loop;
    fdt_transport_t tr = fdt_loop_transport(&loop);
    fdt_inj_t inj;
    sink_t sink = {0};

    assert(fdt_inj_init(&inj, &tr, 4, 8.0, 12345) == 0);

    for (unsigned v = 0; v < 4; v++) {
        char topic[64];
        assert(fdt_topic(topic, sizeof topic, FDT_TOPIC_LSDT, v) == 0);
        assert(tr.subscribe(tr.self, topic, on_msg, &sink) == 0);
    }

    assert(fdt_inj_tick(&inj) == 4);
    assert(tr.poll(tr.self, 0) == 4);
    assert(sink.msgs == 4);
    assert(sink.decoded == 4);   /* every frame survived the round trip */
    assert(fdt_inj_published(&inj) == 4);

    /* The sequence advances per tick, which is what lets the framesync
     * monitor tell a fresh packet from a late one. */
    const uint32_t seq_first = sink.last_seq;
    assert(fdt_inj_tick(&inj) == 4);
    assert(tr.poll(tr.self, 0) == 4);
    assert(sink.last_seq == seq_first + 1);
    assert(fdt_inj_published(&inj) == 8);
}

/** Telemetry sits in the ranges Section II describes for the Jundia boat. */
static void test_plausible_telemetry(void)
{
    fdt_loop_t loop;
    fdt_transport_t tr = fdt_loop_transport(&loop);
    fdt_inj_t inj;
    fdt_input_t in;

    assert(fdt_inj_init(&inj, &tr, 2, 8.0, 42) == 0);
    fdt_inj_sample(&inj, 0, &in);

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
    fdt_inj_sample(&inj, 99, &none);
    assert(none.vbat_v == 0.0f);
}

/** Same seed, same telemetry: a campaign has to be repeatable. */
static void test_deterministic(void)
{
    fdt_loop_t loop_a;
    fdt_loop_t loop_b;
    fdt_transport_t ta = fdt_loop_transport(&loop_a);
    fdt_transport_t tb = fdt_loop_transport(&loop_b);
    fdt_inj_t a;
    fdt_inj_t b;

    assert(fdt_inj_init(&a, &ta, 2, 8.0, 777) == 0);
    assert(fdt_inj_init(&b, &tb, 2, 8.0, 777) == 0);

    for (int t = 0; t < 5; t++) {
        for (unsigned v = 0; v < 2; v++) {
            fdt_input_t ia;
            fdt_input_t ib;
            fdt_inj_sample(&a, v, &ia);
            fdt_inj_sample(&b, v, &ib);
            assert(memcmp(&ia, &ib, sizeof ia) == 0);
        }
        assert(fdt_inj_tick(&a) == 2);
        assert(fdt_inj_tick(&b) == 2);
    }

    /* Different seeds diverge, or the seed would be decorative. */
    fdt_loop_t loop_c;
    fdt_transport_t tc = fdt_loop_transport(&loop_c);
    fdt_inj_t c;
    assert(fdt_inj_init(&c, &tc, 2, 8.0, 778) == 0);

    fdt_input_t ia;
    fdt_input_t ic;
    fdt_inj_sample(&a, 0, &ia);
    fdt_inj_sample(&c, 0, &ic);
    assert(memcmp(&ia, &ic, sizeof ia) != 0);
}

/** A refused publish is counted, never retried. */
static void test_refusal_is_measured(void)
{
    fdt_loop_t loop;
    fdt_transport_t tr = fdt_loop_transport(&loop);
    fdt_inj_t inj;

    /* More vessels than the loopback queue holds, and nothing polls, so the
     * transport starts refusing partway through. Retrying would hide exactly
     * the congestion the scaling benchmark exists to find. */
    assert(fdt_inj_init(&inj, &tr, FDT_LOOP_QUEUE + 8, 8.0, 1) == 0);

    const int sent = fdt_inj_tick(&inj);
    assert(sent == FDT_LOOP_QUEUE);
    assert(fdt_inj_refused(&inj) == 8);

    fdt_inj_note_missed_deadline(&inj);
    assert(fdt_inj_missed_deadlines(&inj) == 1);
}

int main(void)
{
    test_init_guards();
    test_publishes_one_per_vessel();
    test_plausible_telemetry();
    test_deterministic();
    test_refusal_is_measured();

    printf("test_injector: ok\n");
    return 0;
}
