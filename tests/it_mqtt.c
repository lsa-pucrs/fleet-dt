/**
 * @file it_mqtt.c
 * @brief Integration check: the MQTT transport against a real broker.
 */
#include "mqtt/fdt_mqtt.h"

#include <fleet_dt/assembler.h>
#include <fleet_dt/codec.h>
#include <fleet_dt/envelope.h>
#include <fleet_dt/regpub.h>
#include <fleet_dt/relay.h>
#include <fleet_dt/transport.h>

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** What came back off the broker. */
typedef struct {
    unsigned    msgs;
    fdt_state_t decoded;
    uint32_t    seq;
} sink_t;

static void on_state(const char *topic, const uint8_t *buf, size_t len,
                     void *user)
{
    (void)topic;

    sink_t *s = (sink_t *)user;

    fdt_env_t env;
    const uint8_t *payload = NULL;
    if (fdt_env_decode(buf, len, &env, &payload) < 0 || payload == NULL) {
        return;
    }
    if (env.kind != FDT_ENV_STATE) {
        return;
    }
    if (fdt_dec_state(payload, env.payload_len, &s->decoded) < 0) {
        return;
    }

    s->seq = env.seq;
    s->msgs++;
}

/** The twin side of the regulated link, on the ground broker. */
typedef struct {
    fdt_rx_t      rx;
    fdt_rx_slot_t slots[1];
    unsigned      msgs;
    unsigned      raw_seen; /* raw samples that leaked across the bridge */
} rx_sink_t;

static void on_lsdt(const char *topic, const uint8_t *buf, size_t len,
                    void *user)
{
    (void)topic;
    rx_sink_t *s = (rx_sink_t *)user;
    if (fdt_rx_accept(&s->rx, buf, len) >= 0) {
        s->msgs++;
    }
}

static void on_raw_leak(const char *topic, const uint8_t *buf, size_t len,
                        void *user)
{
    (void)topic;
    (void)buf;
    (void)len;
    ((rx_sink_t *)user)->raw_seen++;
}

/** Polls both connections for about @p ms milliseconds. */
static void pump(fdt_transport_t *a, fdt_transport_t *b, int ms)
{
    for (int i = 0; i < ms / 10; i++) {
        (void)a->poll(a->self, 5);
        if (b != a) {
            (void)b->poll(b->self, 5);
        }
    }
}

/**
 * Section III through real brokers. The autopilot publishes every sample on
 * fleet/0/raw/<sensor> to the boat broker: a 1 kHz IMU, a 50 Hz barometer,
 * and 10 Hz GPS and power, for one 125 ms tick. The regulator of the boat
 * broker (fdt_relay_t) holds the newest sample of each sensor and releases
 * it at the edge on fleet/0/lsdt/<sensor>. The twin subscribes on the ground
 * broker. With two brokers, the bridge of config/mosquitto/boat.conf is the
 * only path between them, and the raw level must not cross it.
 */
static int regulated_round_trip(fdt_transport_t *boat, fdt_transport_t *ground)
{
    static rx_sink_t sink; /* outlives the subscriptions */
    assert(fdt_rx_init(&sink.rx, sink.slots, 1, NULL) == 0);
    assert(ground->subscribe(ground->self, FDT_TOPIC_LSDT_ALL, on_lsdt,
                             &sink) == 0);
    if (ground != boat) {
        assert(ground->subscribe(ground->self, FDT_TOPIC_RAW_ALL,
                                 on_raw_leak, &sink) == 0);
    }

    const double hz[FDT_SENSOR_COUNT] = { 1000.0, 10.0, 50.0, 10.0 };
    static fdt_regpub_t stage[1];
    static fdt_relay_t  relay;
    assert(fdt_relay_init(&relay, boat, stage, 1, hz) == 0);
    pump(boat, ground, 300);

    fdt_regpub_t autopilot;
    assert(fdt_regpub_init(&autopilot, boat, 0, hz, 1) == 0);
    assert(fdt_regpub_set_level(&autopilot, FDT_TOPIC_RAW) == 0);

    for (int s = 0; s < FDT_SENSOR_COUNT; s++) {
        const unsigned m = fdt_reg_samples_in_tick(hz[s], 8.0, 0);
        for (unsigned j = 1; j <= m; j++) {
            fdt_input_t in = {0};
            in.wz_rps      = 0.001f * (float)j;  /* IMU: newest is j = 125 */
            in.gps_lat_deg = -30.0f - 0.01f * (float)j;
            in.press_pa    = 101000.0f + (float)j;
            in.vbat_v      = 18.0f + 0.1f * (float)j;
            assert(fdt_regpub_offer(&autopilot, (fdt_sensor_t)s, &in) == 1);
        }
    }

    /* The raw samples reach the regulator through the boat broker. */
    for (int i = 0; i < 1000 && fdt_relay_received(&relay) < 133; i++) {
        (void)boat->poll(boat->self, 10);
    }
    if (fdt_relay_received(&relay) != 133) {
        fprintf(stderr, "boat broker: %llu of 133 raw samples reached the "
                        "regulator\n",
                (unsigned long long)fdt_relay_received(&relay));
        return -1;
    }

    assert(fdt_relay_tick(&relay) == FDT_SENSOR_COUNT);

    for (int i = 0; i < 200 && sink.msgs < FDT_SENSOR_COUNT; i++) {
        pump(boat, ground, 10);
    }
    pump(boat, ground, 200); /* let any leaked raw sample arrive */
    if (sink.msgs != FDT_SENSOR_COUNT) {
        fprintf(stderr, "regulated tick: %u of %d messages came back\n",
                sink.msgs, FDT_SENSOR_COUNT);
        return -1;
    }
    if (sink.raw_seen != 0) {
        fprintf(stderr, "bridge leaked %u raw samples\n", sink.raw_seen);
        return -1;
    }

    const fdt_input_t *got = fdt_rx_input(&sink.rx, 0);
    assert(got->wz_rps == 0.001f * 125.0f);
    assert(got->gps_lat_deg == -30.0f - 0.01f * 1.0f);
    assert(got->press_pa == 101000.0f + 6.0f);
    assert(got->vbat_v == 18.0f + 0.1f * 1.0f);

    printf("%s: 133 raw samples into the boat broker, %u regulated "
           "messages out, %u raw across; I^t rebuilt from the newest\n",
           (ground != boat) ? "bridge boat -> ground" : "one broker",
           sink.msgs, sink.raw_seen);
    return 0;
}

int main(int argc, char **argv)
{
    const int port = (argc > 1) ? atoi(argv[1]) : 1883;
    /* A second port names the boat broker, bridged to the first. */
    const int boat_port = (argc > 2) ? atoi(argv[2]) : 0;

    fdt_transport_t tr;
    if (fdt_mqtt_open("127.0.0.1", port, "fleet-dt-it", &tr) != 0) {
        fprintf(stderr, "cannot reach a broker on 127.0.0.1:%d\n", port);
        return EXIT_FAILURE;
    }
    printf("connected to 127.0.0.1:%d\n", port);

    char topic[64];
    assert(fdt_topic(topic, sizeof topic, FDT_TOPIC_STATE, 0) == 0);

    sink_t sink = {0};
    assert(tr.subscribe(tr.self, topic, on_state, &sink) == 0);

    for (int i = 0; i < 20; i++) {
        (void)tr.poll(tr.self, 50);
    }

    /* A state whose values are recognisable on the far side. */
    const fdt_state_t sent = {
        .lat_deg = -30.05f, .lon_deg = -51.17f, .alt_m = 3.0f,
        .roll_deg = 1.5f, .pitch_deg = -2.5f, .yaw_deg = 91.25f,
        .surge_mps = 1.25f, .sway_mps = -0.5f, .heave_mps = 0.0f,
        .roll_rate_rps = 0.1f, .pitch_rate_rps = -0.2f, .yaw_rate_rps = 0.3f,
    };

    uint8_t payload[FDT_WIRE_STATE_BYTES];
    assert(fdt_enc_state(&sent, payload, sizeof payload) ==
           FDT_WIRE_STATE_BYTES);

    const fdt_env_t env = { .magic = FDT_ENV_MAGIC, .vessel = 0,
                            .kind = FDT_ENV_STATE, .flags = 0, .seq = 4242,
                            .payload_len = FDT_WIRE_STATE_BYTES };
    uint8_t frame[FDT_ENV_HEADER_BYTES + FDT_WIRE_STATE_BYTES];
    const long n = fdt_env_encode(&env, payload, sizeof payload,
                                  frame, sizeof frame);
    assert(n > 0);

    assert(tr.publish(tr.self, topic, frame, (size_t)n, 1) == 0);

    for (int i = 0; i < 40 && sink.msgs == 0; i++) {
        (void)tr.poll(tr.self, 50);
    }

    if (sink.msgs == 0) {
        fprintf(stderr, "published but nothing came back\n");
        fdt_mqtt_close(&tr);
        return EXIT_FAILURE;
    }

    /* Byte-for-byte: the state that left is the state that arrived. */
    assert(sink.seq == 4242);
    assert(memcmp(&sent, &sink.decoded, sizeof sent) == 0);

    printf("round trip over mosquitto: seq %u, %d bytes, state identical\n",
           sink.seq, FDT_WIRE_STATE_BYTES);

    fdt_transport_t boat = tr;
    if (boat_port > 0) {
        if (fdt_mqtt_open("127.0.0.1", boat_port, "fleet-dt-it-boat",
                          &boat) != 0) {
            fprintf(stderr, "cannot reach the boat broker on 127.0.0.1:%d\n",
                    boat_port);
            fdt_mqtt_close(&tr);
            return EXIT_FAILURE;
        }
        printf("connected to the boat broker on 127.0.0.1:%d\n", boat_port);
    }

    const int rc = regulated_round_trip(&boat, &tr);
    if (boat_port > 0) {
        fdt_mqtt_close(&boat);
    }
    if (rc != 0) {
        fdt_mqtt_close(&tr);
        return EXIT_FAILURE;
    }

    fdt_mqtt_close(&tr);
    assert(tr.self == NULL);

    printf("it_mqtt: ok\n");
    return EXIT_SUCCESS;
}
