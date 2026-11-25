#ifndef FLEET_DT_MQTT_WIRE_H
#define FLEET_DT_MQTT_WIRE_H

#include <stddef.h>
#include <stdint.h>

/**
 * @file fdt_mqtt_wire.h
 * @brief MQTT 5.0 packet encoding, as much of it as the fleet uses.
 *
 * Abstract: the architecture "relies on MQTT (Message Queuing Telemetry
 * Transport) protocol". The client of fdt_mqtt.c speaks it over a plain
 * socket with these functions, so the repository needs no MQTT library.
 * Each encoder writes one whole control packet and returns its length, or
 * -1 when @p cap is too small or an argument is out of range. No properties
 * are sent: every property length is 0.
 */

/** MQTT control packet types (high nibble of the fixed header). */
enum {
    FDT_MQTT_CONNECT    = 1,
    FDT_MQTT_CONNACK    = 2,
    FDT_MQTT_PUBLISH    = 3,
    FDT_MQTT_PUBACK     = 4,
    FDT_MQTT_SUBSCRIBE  = 8,
    FDT_MQTT_SUBACK     = 9,
    FDT_MQTT_PINGREQ    = 12,
    FDT_MQTT_PINGRESP   = 13,
    FDT_MQTT_DISCONNECT = 14
};

/** @brief Writes a Variable Byte Integer. @return Bytes written, or -1. */
int fdt_mqtt_put_varint(uint8_t *p, size_t cap, uint32_t v);

/**
 * @brief Reads a Variable Byte Integer.
 * @return Bytes consumed; 0 when more bytes are needed; -1 when malformed.
 */
int fdt_mqtt_get_varint(const uint8_t *p, size_t len, uint32_t *v);

/** @brief CONNECT, protocol level 5, clean start. */
long fdt_mqtt_enc_connect(uint8_t *p, size_t cap, const char *client_id,
                          uint16_t keepalive_s);

/** @brief PUBLISH; @p packet_id is ignored for QoS 0. */
long fdt_mqtt_enc_publish(uint8_t *p, size_t cap, const char *topic,
                          const uint8_t *payload, size_t len, int qos,
                          uint16_t packet_id);

/** @brief PUBACK, reason Success. */
long fdt_mqtt_enc_puback(uint8_t *p, size_t cap, uint16_t packet_id);

/** @brief SUBSCRIBE to one topic filter at @p qos. */
long fdt_mqtt_enc_subscribe(uint8_t *p, size_t cap, uint16_t packet_id,
                            const char *filter, int qos);

/** @brief PINGREQ. */
long fdt_mqtt_enc_pingreq(uint8_t *p, size_t cap);

/** @brief DISCONNECT, reason Normal. */
long fdt_mqtt_enc_disconnect(uint8_t *p, size_t cap);

/** One received control packet, pointing into the caller's buffer. */
typedef struct {
    int            type;
    int            flags;      /**< Low nibble of the fixed header. */
    size_t         total;      /**< Bytes the whole packet occupies. */
    const uint8_t *body;       /**< Variable header and payload. */
    size_t         body_len;
} fdt_mqtt_packet_t;

/**
 * @brief Frames the next packet at the head of @p buf.
 * @return 1 with @p out filled; 0 when the packet is not complete yet;
 *         -1 when the fixed header is malformed.
 */
int fdt_mqtt_frame(const uint8_t *buf, size_t len, fdt_mqtt_packet_t *out);

/** A PUBLISH taken apart. */
typedef struct {
    const char    *topic;      /**< Not NUL-terminated; see topic_len. */
    size_t         topic_len;
    int            qos;
    uint16_t       packet_id;  /**< 0 for QoS 0. */
    const uint8_t *payload;
    size_t         payload_len;
} fdt_mqtt_publish_t;

/** @brief Parses a framed PUBLISH. @return 0, or -1 when malformed. */
int fdt_mqtt_parse_publish(const fdt_mqtt_packet_t *pk,
                           fdt_mqtt_publish_t *out);

/**
 * @brief The reason code of a CONNACK, PUBACK or SUBACK (first granted QoS
 *        for SUBACK).
 * @return The code (0 is success, or the granted QoS 0..2), or -1.
 */
int fdt_mqtt_ack_reason(const fdt_mqtt_packet_t *pk);

#endif /* FLEET_DT_MQTT_WIRE_H */
