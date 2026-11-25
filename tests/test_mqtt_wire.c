/**
 * @file test_mqtt_wire.c
 * @brief The MQTT 5.0 packets of the repository's own client, byte for byte.
 */
#include "mqtt/fdt_mqtt_wire.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void test_varint(void)
{
    /* The boundaries of MQTT 5.0, section 1.5.5. */
    static const struct { uint32_t v; int n; } V[] = {
        { 0, 1 }, { 127, 1 }, { 128, 2 }, { 16383, 2 },
        { 16384, 3 }, { 2097151, 3 }, { 2097152, 4 }, { 268435455, 4 },
    };
    for (size_t i = 0; i < sizeof V / sizeof V[0]; i++) {
        uint8_t b[4];
        uint32_t back = 0;
        assert(fdt_mqtt_put_varint(b, sizeof b, V[i].v) == V[i].n);
        assert(fdt_mqtt_get_varint(b, (size_t)V[i].n, &back) == V[i].n);
        assert(back == V[i].v);
        /* One byte short: the reader asks for more. */
        assert(fdt_mqtt_get_varint(b, (size_t)V[i].n - 1, &back) == 0);
    }
    uint8_t b[4];
    assert(fdt_mqtt_put_varint(b, sizeof b, 268435456u) == -1);
    assert(fdt_mqtt_put_varint(b, 1, 128) == -1);
    const uint8_t bad[5] = { 0xFF, 0xFF, 0xFF, 0xFF, 0x01 };
    uint32_t v;
    assert(fdt_mqtt_get_varint(bad, sizeof bad, &v) == -1);
}

static void test_connect_bytes(void)
{
    uint8_t p[64];
    const long n = fdt_mqtt_enc_connect(p, sizeof p, "boat1", 60);
    static const uint8_t want[] = {
        0x10, 18,                           /* CONNECT, remaining length */
        0x00, 0x04, 'M', 'Q', 'T', 'T',     /* protocol name */
        0x05,                               /* protocol level 5 */
        0x02,                               /* clean start */
        0x00, 0x3C,                         /* keepalive 60 s */
        0x00,                               /* properties */
        0x00, 0x05, 'b', 'o', 'a', 't', '1' /* client id */
    };
    assert(n == (long)sizeof want);
    assert(memcmp(p, want, sizeof want) == 0);
    assert(fdt_mqtt_enc_connect(p, 10, "boat1", 60) == -1);
}

static void test_publish_round_trip(void)
{
    uint8_t p[256];
    const uint8_t payload[56] = { 1, 2, 3 };
    const long n = fdt_mqtt_enc_publish(p, sizeof p, "fleet/1/lsdt/imu",
                                        payload, sizeof payload, 1, 0x1234);
    /* 1 + 1 (length) + 2 + 16 (topic) + 2 (id) + 1 (properties) + 56 */
    assert(n == 79);
    assert(p[0] == 0x32); /* PUBLISH, QoS 1 */

    fdt_mqtt_packet_t pk;
    assert(fdt_mqtt_frame(p, (size_t)n - 1, &pk) == 0); /* incomplete */
    assert(fdt_mqtt_frame(p, (size_t)n, &pk) == 1);
    assert(pk.type == FDT_MQTT_PUBLISH && pk.total == (size_t)n);

    fdt_mqtt_publish_t m;
    assert(fdt_mqtt_parse_publish(&pk, &m) == 0);
    assert(m.qos == 1 && m.packet_id == 0x1234);
    assert(m.topic_len == 16 && memcmp(m.topic, "fleet/1/lsdt/imu", 16) == 0);
    assert(m.payload_len == sizeof payload && m.payload[2] == 3);

    /* QoS 0 carries no packet identifier. */
    const long z = fdt_mqtt_enc_publish(p, sizeof p, "t", payload, 2, 0, 0);
    assert(z == 1 + 1 + 3 + 1 + 2 && p[0] == 0x30);
    assert(fdt_mqtt_frame(p, (size_t)z, &pk) == 1);
    assert(fdt_mqtt_parse_publish(&pk, &m) == 0 && m.packet_id == 0);

    /* QoS 1 needs a non-zero identifier; QoS 2 is not used by the fleet. */
    assert(fdt_mqtt_enc_publish(p, sizeof p, "t", payload, 2, 1, 0) == -1);
    assert(fdt_mqtt_enc_publish(p, sizeof p, "t", payload, 2, 2, 7) == -1);

    printf("PUBLISH of one 56-byte IMU message: %ld bytes on the wire\n", n);
}

static void test_subscribe_and_acks(void)
{
    uint8_t p[64];
    const long n = fdt_mqtt_enc_subscribe(p, sizeof p, 7, "fleet/+/lsdt/+", 1);
    assert(n > 0);
    assert(p[0] == 0x82); /* SUBSCRIBE with the reserved flags 0b0010 */

    /* A SUBACK granting QoS 1, and a PUBACK with its reason omitted. */
    const uint8_t suback[] = { 0x90, 0x04, 0x00, 0x07, 0x00, 0x01 };
    fdt_mqtt_packet_t pk;
    assert(fdt_mqtt_frame(suback, sizeof suback, &pk) == 1);
    assert(fdt_mqtt_ack_reason(&pk) == 1);

    assert(fdt_mqtt_enc_puback(p, sizeof p, 9) == 4);
    assert(fdt_mqtt_frame(p, 4, &pk) == 1 && pk.type == FDT_MQTT_PUBACK);
    assert(fdt_mqtt_ack_reason(&pk) == 0);

    const uint8_t connack[] = { 0x20, 0x03, 0x00, 0x00, 0x00 };
    assert(fdt_mqtt_frame(connack, sizeof connack, &pk) == 1);
    assert(fdt_mqtt_ack_reason(&pk) == 0);

    assert(fdt_mqtt_enc_pingreq(p, sizeof p) == 2 && p[0] == 0xC0);
    assert(fdt_mqtt_enc_disconnect(p, sizeof p) == 2 && p[0] == 0xE0);
}

int main(void)
{
    test_varint();
    test_connect_bytes();
    test_publish_round_trip();
    test_subscribe_and_acks();

    printf("test_mqtt_wire: ok\n");
    return 0;
}
