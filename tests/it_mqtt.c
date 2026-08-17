/**
 * @file it_mqtt.c
 * @brief Integration check: the MQTT transport against a real broker.
 *
 * The unit suite exercises the transport seam over the in-process loopback,
 * which proves the seam and proves nothing about mosquitto. This one connects
 * to a broker, publishes an envelope, polls it back and decodes it, which is
 * the difference between an adapter that compiles and an adapter that works.
 *
 * Not part of `make test`: it needs libmosquitto and a running broker. Run it
 * with `make mqtt-test`, which starts a broker on a port of its own so it
 * cannot collide with one already serving a fleet.
 *
 * Claim covered: C2, the half the loopback cannot reach.
 */
#include "mqtt/fdt_mqtt.h"

#include <fleet_dt/codec.h>
#include <fleet_dt/envelope.h>
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

int main(int argc, char **argv)
{
    const int port = (argc > 1) ? atoi(argv[1]) : 1883;

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

    /* The subscription is acknowledged asynchronously, so poll until the
     * broker has it before publishing anything to it. */
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

    fdt_mqtt_close(&tr);
    assert(tr.self == NULL);

    printf("it_mqtt: ok\n");
    return EXIT_SUCCESS;
}
