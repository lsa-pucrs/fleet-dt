#include "fdt_mqtt_wire.h"

#include <string.h>

int fdt_mqtt_put_varint(uint8_t *p, size_t cap, uint32_t v)
{
    if (v > 268435455u) {
        return -1;
    }
    size_t n = 0;
    do {
        if (n >= cap) {
            return -1;
        }
        uint8_t b = (uint8_t)(v % 128u);
        v /= 128u;
        if (v > 0) {
            b |= 0x80u;
        }
        p[n++] = b;
    } while (v > 0);
    return (int)n;
}

int fdt_mqtt_get_varint(const uint8_t *p, size_t len, uint32_t *v)
{
    uint32_t value = 0;
    uint32_t mult = 1;
    for (size_t i = 0; i < 4; i++) {
        if (i >= len) {
            return 0;
        }
        value += (uint32_t)(p[i] & 0x7Fu) * mult;
        if ((p[i] & 0x80u) == 0) {
            *v = value;
            return (int)(i + 1);
        }
        mult *= 128u;
    }
    return -1;
}

/** Builds fixed header + body into p; body is assembled at p + 5 first. */
static long finish(uint8_t *p, size_t cap, uint8_t first, size_t body)
{
    uint8_t hdr[5];
    hdr[0] = first;
    const int vn = fdt_mqtt_put_varint(hdr + 1, 4, (uint32_t)body);
    if (vn < 0 || 1 + (size_t)vn + body > cap) {
        return -1;
    }
    memmove(p + 1 + vn, p + 5, body);
    memcpy(p, hdr, 1 + (size_t)vn);
    return (long)(1 + (size_t)vn + body);
}

static int put_str(uint8_t *p, size_t cap, size_t *at, const char *s,
                   size_t n)
{
    if (n > 0xFFFFu || *at + 2 + n > cap) {
        return -1;
    }
    p[(*at)++] = (uint8_t)(n >> 8);
    p[(*at)++] = (uint8_t)n;
    memcpy(p + *at, s, n);
    *at += n;
    return 0;
}

long fdt_mqtt_enc_connect(uint8_t *p, size_t cap, const char *client_id,
                          uint16_t keepalive_s)
{
    if (p == NULL || client_id == NULL || cap < 5) {
        return -1;
    }
    size_t at = 5;
    if (put_str(p, cap, &at, "MQTT", 4) != 0 || at + 4 > cap) {
        return -1;
    }
    p[at++] = 5;      /* protocol level: MQTT 5.0 */
    p[at++] = 0x02;   /* clean start */
    p[at++] = (uint8_t)(keepalive_s >> 8);
    p[at++] = (uint8_t)keepalive_s;
    if (at + 1 > cap) {
        return -1;
    }
    p[at++] = 0;      /* properties */
    if (put_str(p, cap, &at, client_id, strlen(client_id)) != 0) {
        return -1;
    }
    return finish(p, cap, (uint8_t)(FDT_MQTT_CONNECT << 4), at - 5);
}

long fdt_mqtt_enc_publish(uint8_t *p, size_t cap, const char *topic,
                          const uint8_t *payload, size_t len, int qos,
                          uint16_t packet_id)
{
    if (p == NULL || topic == NULL || (payload == NULL && len > 0) ||
        qos < 0 || qos > 1 || cap < 5) {
        return -1;
    }
    size_t at = 5;
    if (put_str(p, cap, &at, topic, strlen(topic)) != 0) {
        return -1;
    }
    if (qos > 0) {
        if (at + 2 > cap || packet_id == 0) {
            return -1;
        }
        p[at++] = (uint8_t)(packet_id >> 8);
        p[at++] = (uint8_t)packet_id;
    }
    if (at + 1 + len > cap) {
        return -1;
    }
    p[at++] = 0;      /* properties */
    if (len > 0) {
        memcpy(p + at, payload, len);
    }
    at += len;
    return finish(p, cap, (uint8_t)((FDT_MQTT_PUBLISH << 4) | (qos << 1)),
                  at - 5);
}

long fdt_mqtt_enc_puback(uint8_t *p, size_t cap, uint16_t packet_id)
{
    if (p == NULL || cap < 4) {
        return -1;
    }
    /* Reason Success with no properties may be omitted (MQTT 5, 3.4.2.1). */
    p[0] = (uint8_t)(FDT_MQTT_PUBACK << 4);
    p[1] = 2;
    p[2] = (uint8_t)(packet_id >> 8);
    p[3] = (uint8_t)packet_id;
    return 4;
}

long fdt_mqtt_enc_subscribe(uint8_t *p, size_t cap, uint16_t packet_id,
                            const char *filter, int qos)
{
    if (p == NULL || filter == NULL || qos < 0 || qos > 2 || cap < 8 ||
        packet_id == 0) {
        return -1;
    }
    size_t at = 5;
    p[at++] = (uint8_t)(packet_id >> 8);
    p[at++] = (uint8_t)packet_id;
    p[at++] = 0;      /* properties */
    if (put_str(p, cap, &at, filter, strlen(filter)) != 0 || at + 1 > cap) {
        return -1;
    }
    p[at++] = (uint8_t)qos;
    /* SUBSCRIBE carries the reserved flags 0b0010. */
    return finish(p, cap, (uint8_t)((FDT_MQTT_SUBSCRIBE << 4) | 0x02),
                  at - 5);
}

long fdt_mqtt_enc_pingreq(uint8_t *p, size_t cap)
{
    if (p == NULL || cap < 2) {
        return -1;
    }
    p[0] = (uint8_t)(FDT_MQTT_PINGREQ << 4);
    p[1] = 0;
    return 2;
}

long fdt_mqtt_enc_disconnect(uint8_t *p, size_t cap)
{
    if (p == NULL || cap < 2) {
        return -1;
    }
    p[0] = (uint8_t)(FDT_MQTT_DISCONNECT << 4);
    p[1] = 0;         /* Normal disconnection, reason omitted */
    return 2;
}

int fdt_mqtt_frame(const uint8_t *buf, size_t len, fdt_mqtt_packet_t *out)
{
    if (buf == NULL || out == NULL || len < 2) {
        return 0;
    }
    uint32_t rem = 0;
    const int vn = fdt_mqtt_get_varint(buf + 1, len - 1, &rem);
    if (vn < 0) {
        return -1;
    }
    if (vn == 0 || len < 1 + (size_t)vn + rem) {
        return 0;
    }
    out->type     = buf[0] >> 4;
    out->flags    = buf[0] & 0x0F;
    out->body     = buf + 1 + vn;
    out->body_len = rem;
    out->total    = 1 + (size_t)vn + rem;
    return 1;
}

int fdt_mqtt_parse_publish(const fdt_mqtt_packet_t *pk,
                           fdt_mqtt_publish_t *out)
{
    if (pk == NULL || out == NULL || pk->type != FDT_MQTT_PUBLISH ||
        pk->body_len < 2) {
        return -1;
    }
    const uint8_t *b = pk->body;
    size_t at = 0;
    const size_t tl = ((size_t)b[0] << 8) | b[1];
    at = 2;
    if (at + tl > pk->body_len) {
        return -1;
    }
    out->topic     = (const char *)(b + at);
    out->topic_len = tl;
    at += tl;

    out->qos = (pk->flags >> 1) & 0x03;
    out->packet_id = 0;
    if (out->qos > 0) {
        if (at + 2 > pk->body_len) {
            return -1;
        }
        out->packet_id = (uint16_t)(((unsigned)b[at] << 8) | b[at + 1]);
        at += 2;
    }

    uint32_t plen = 0;
    const int vn = fdt_mqtt_get_varint(b + at, pk->body_len - at, &plen);
    if (vn <= 0 || at + (size_t)vn + plen > pk->body_len) {
        return -1;
    }
    at += (size_t)vn + plen; /* properties skipped */

    out->payload     = b + at;
    out->payload_len = pk->body_len - at;
    return 0;
}

int fdt_mqtt_ack_reason(const fdt_mqtt_packet_t *pk)
{
    if (pk == NULL) {
        return -1;
    }
    switch (pk->type) {
    case FDT_MQTT_CONNACK:
        /* ack flags, reason code */
        return (pk->body_len >= 2) ? pk->body[1] : -1;
    case FDT_MQTT_PUBACK:
        /* packet id, then reason; absent reason means Success */
        return (pk->body_len >= 3) ? pk->body[2] : 0;
    case FDT_MQTT_SUBACK: {
        if (pk->body_len < 3) {
            return -1;
        }
        uint32_t plen = 0;
        const int vn = fdt_mqtt_get_varint(pk->body + 2, pk->body_len - 2,
                                           &plen);
        const size_t at = 2 + (size_t)((vn > 0) ? vn : 0) + plen;
        return (vn > 0 && at < pk->body_len) ? pk->body[at] : -1;
    }
    default:
        return -1;
    }
}
