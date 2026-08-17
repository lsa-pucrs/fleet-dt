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

#endif /* FLEET_DT_CODEC_H */
