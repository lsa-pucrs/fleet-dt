#ifndef FLEET_DT_MAVLINK_WIRE_H
#define FLEET_DT_MAVLINK_WIRE_H

#include "mavlink/fdt_mavlink.h"

#include <stddef.h>
#include <stdint.h>

/**
 * @file fdt_mavlink_wire.h
 * @brief MAVLink framing for the Ardupilot link of Figure 3, without a
 *        MAVLink library.
 *
 * Figure 3: "Ardupilot feeds sensor data (I_k) to the update function ...
 * Ardupilot receives only the actuation state." Ardupilot speaks MAVLink.
 * This module frames MAVLink 1 and 2 packets (CRC-16/MCRF4XX, "X.25", with
 * the per-message CRC_EXTRA seed), decodes the five messages that carry the
 * telemetry of Table I into fdt_mav_ingest_t, and encodes the HEARTBEAT a
 * ground station announces itself with and the RC_CHANNELS_OVERRIDE that
 * carries A^t back. Field order and CRC_EXTRA follow common.xml.
 */

/** Message identifiers used here (common.xml). */
enum {
    FDT_MAVMSG_HEARTBEAT            = 0,
    FDT_MAVMSG_GPS_RAW_INT          = 24,
    FDT_MAVMSG_SCALED_PRESSURE      = 29,
    FDT_MAVMSG_LOCAL_POSITION_NED   = 32,
    FDT_MAVMSG_RC_CHANNELS_OVERRIDE = 70,
    FDT_MAVMSG_SCALED_IMU2          = 116,
    FDT_MAVMSG_BATTERY_STATUS       = 147
};

/** Largest payload MAVLink allows. */
#define FDT_MAV_MAX_PAYLOAD 255

/** Largest frame: MAVLink 2 header, payload, CRC and signature. */
#define FDT_MAV_MAX_FRAME (10 + FDT_MAV_MAX_PAYLOAD + 2 + 13)

/** One received message; the payload is zero-extended to its full length,
 *  undoing MAVLink 2's trailing-zero truncation. */
typedef struct {
    uint32_t msgid;
    uint8_t  sysid;
    uint8_t  compid;
    uint8_t  seq;
    uint8_t  version;  /**< 1 or 2. */
    uint8_t  len;      /**< Bytes on the wire, before zero extension. */
    uint8_t  payload[FDT_MAV_MAX_PAYLOAD];
} fdt_mav_msg_t;

/** A byte-stream parser. Zero it, then feed bytes. */
typedef struct {
    uint8_t  buf[FDT_MAV_MAX_FRAME];
    size_t   have;
    uint64_t frames;     /**< Frames whose CRC matched. */
    uint64_t bad_crc;    /**< Frames dropped on a CRC mismatch. */
    uint64_t unknown;    /**< Frames of messages without a known CRC_EXTRA. */
} fdt_mav_parser_t;

/** @brief CRC-16/MCRF4XX over @p n bytes, starting from @p crc. */
uint16_t fdt_mav_crc(uint16_t crc, const uint8_t *p, size_t n);

/**
 * @brief The CRC_EXTRA seed of a message.
 * @return The seed, or -1 for a message this module does not know.
 */
int fdt_mav_crc_extra(uint32_t msgid);

/**
 * @brief Feeds one byte.
 * @return 1 when @p out holds a complete, CRC-checked message; 0 otherwise.
 *         Frames of unknown messages are skipped, since their CRC cannot be
 *         checked without the seed.
 */
int fdt_mav_parse(fdt_mav_parser_t *p, uint8_t byte, fdt_mav_msg_t *out);

/**
 * @brief Folds a telemetry message into the frame being assembled.
 * @return 1 when the message carried Table I telemetry, 0 when it is of
 *         another kind.
 */
int fdt_mav_dispatch(fdt_mav_ingest_t *ing, const fdt_mav_msg_t *msg);

/**
 * @brief Encodes a MAVLink 2 HEARTBEAT, as a ground control station.
 * @return Frame length, or -1 when @p cap is too small.
 */
long fdt_mav_enc_heartbeat(uint8_t *buf, size_t cap, uint8_t seq,
                           uint8_t sysid, uint8_t compid);

/**
 * @brief Encodes A^t as a MAVLink 2 RC_CHANNELS_OVERRIDE.
 *
 * Throttle and cage go on the channels Ardupilot Rover reads as throttle
 * (3) and steering (1), through fdt_mav_actuation_to_rc(). Every other
 * channel is left as it is: UINT16_MAX on channels 1 to 8, 0 beyond.
 *
 * @return Frame length, or -1 when @p cap is too small or @p a is NULL.
 */
long fdt_mav_enc_rc_override(uint8_t *buf, size_t cap, uint8_t seq,
                             uint8_t sysid, uint8_t compid,
                             uint8_t target_sys, uint8_t target_comp,
                             const fdt_actuation_t *a);

#endif /* FLEET_DT_MAVLINK_WIRE_H */
