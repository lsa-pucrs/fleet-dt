#ifndef FLEET_DT_CODEC_H
#define FLEET_DT_CODEC_H

#include <fleet_dt/model.h>

#include <stddef.h>
#include <stdint.h>

/**
 * @file codec.h
 * @brief Fixed-width little-endian wire encoding for I^t, B^t and A^t.
 */

/**
 * Encoded size of B^t: the twelve floats of equation (1).
 */
#define FDT_WIRE_STATE_BYTES 48

/**
 * Encoded size of I^t: 78 bytes.
 */
#define FDT_WIRE_INPUT_BYTES 78

/** Encoded size of A^t: throttle and cage angle. */
#define FDT_WIRE_ACT_BYTES 8

/**
 * Sentinel meaning "a camera view was attached when this input was encoded".
 */
#define FDT_VIEW_PRESENT ((const void *)(uintptr_t)1)

/**
 * @brief Encodes B^t.
 * @param buf  Destination.
 * @param cap  Capacity of @p buf.
 * @return ::FDT_WIRE_STATE_BYTES written, or -1 when an argument is NULL or
 *         @p cap is too small.
 */
long fdt_enc_state(const fdt_state_t *b, uint8_t *buf, size_t cap);

/**
 * @brief Decodes B^t.
 * @param len  Bytes available in @p buf.
 * @return ::FDT_WIRE_STATE_BYTES consumed, or -1 when @p len is short.
 */
long fdt_dec_state(const uint8_t *buf, size_t len, fdt_state_t *b);

/**
 * @brief Encodes I^t, replacing each camera view with a presence flag.
 * @return ::FDT_WIRE_INPUT_BYTES written, or -1 on a short buffer.
 */
long fdt_enc_input(const fdt_input_t *in, uint8_t *buf, size_t cap);

/**
 * @brief Decodes I^t.
 *
 * @return ::FDT_WIRE_INPUT_BYTES consumed, or -1 on a short buffer.
 */
long fdt_dec_input(const uint8_t *buf, size_t len, fdt_input_t *in);

/**
 * @brief Encodes A^t.
 * @return ::FDT_WIRE_ACT_BYTES written, or -1 on a short buffer.
 */
long fdt_enc_act(const fdt_actuation_t *a, uint8_t *buf, size_t cap);

/**
 * @brief Decodes A^t.
 * @return ::FDT_WIRE_ACT_BYTES consumed, or -1 on a short buffer.
 */
long fdt_dec_act(const uint8_t *buf, size_t len, fdt_actuation_t *a);

/**
 * The four sensors of the LSDT group, as the boat publishes them: one message
 * per sensor sample (slide 13). Together they carry every scalar of I^t in
 * Table I; the stereo views travel off MQTT (Section III).
 */
typedef enum {
    FDT_SENSOR_IMU   = 0, /**< a, omega, m: nine floats (NAVIO2 IMU). */
    FDT_SENSOR_GPS   = 1, /**< phi, lambda, h, v_N, v_E, v_D: six floats. */
    FDT_SENSOR_BARO  = 2, /**< P, T: two floats (MS5611). */
    FDT_SENSOR_POWER = 3, /**< V_b, I_b: two floats (Arduino Micro). */
    FDT_SENSOR_COUNT = 4
} fdt_sensor_t;

/** Encoded size of one IMU sample. */
#define FDT_WIRE_IMU_BYTES 36
/** Encoded size of one GPS sample. */
#define FDT_WIRE_GPS_BYTES 24
/** Encoded size of one barometer sample. */
#define FDT_WIRE_BARO_BYTES 8
/** Encoded size of one power sample. */
#define FDT_WIRE_POWER_BYTES 8
/** Largest encoded sensor sample. */
#define FDT_WIRE_SENSOR_MAX_BYTES FDT_WIRE_IMU_BYTES

/** @brief Encoded size of one sample of @p s, or 0 for an unknown sensor. */
size_t fdt_sensor_bytes(fdt_sensor_t s);

/** @brief Short lowercase name of @p s ("imu", "gps", ...), or NULL. */
const char *fdt_sensor_name(fdt_sensor_t s);

/**
 * @brief Encodes the fields of I^t that sensor @p s measures.
 * @return Bytes written, or -1 on a NULL argument, an unknown sensor or a
 *         short buffer.
 */
long fdt_enc_sensor(const fdt_input_t *in, fdt_sensor_t s, uint8_t *buf,
                    size_t cap);

/**
 * @brief Decodes one sample of sensor @p s into the matching fields of
 *        @p in, leaving every other field untouched.
 * @return Bytes consumed, or -1 on a NULL argument, an unknown sensor or a
 *         short buffer.
 */
long fdt_dec_sensor(const uint8_t *buf, size_t len, fdt_sensor_t s,
                    fdt_input_t *in);

#endif /* FLEET_DT_CODEC_H */
