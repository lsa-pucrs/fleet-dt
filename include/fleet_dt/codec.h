#ifndef FLEET_DT_CODEC_H
#define FLEET_DT_CODEC_H

#include <fleet_dt/model.h>

#include <stddef.h>
#include <stdint.h>

/**
 * @file codec.h
 * @brief Fixed-width little-endian wire encoding for I^t, B^t and A^t.
 *
 * A boat runs an ARM Raspberry Pi 4 and the ground station runs x86; both are
 * little-endian today, and neither is guaranteed to be tomorrow. Every field
 * is therefore serialised byte by byte at an explicit width, rather than by
 * copying the struct, so the format is a property of the protocol and not of
 * whichever compiler built the peer.
 *
 * Field order on the wire is the declaration order of the structs in model.h,
 * which is the order equation (1) and Table I list them in.
 */

/**
 * Encoded size of B^t: the twelve floats of equation (1).
 *
 * This equals sizeof(fdt_state_t) and the 48 bytes Section IV publishes, but
 * for a different reason — one is a memory layout, the other a protocol — so
 * the two are asserted separately.
 */
#define FDT_WIRE_STATE_BYTES 48

/**
 * Encoded size of I^t: 78 bytes.
 *
 * Table I lists 21 entries. Two are the stereo camera views, which do not
 * travel here: Section III moves the camera feed off MQTT and onto RTSP. The
 * remaining 19 scalars are 76 bytes, plus one presence flag per view, giving
 * 78. The receiver re-attaches the images from the high-speed path.
 */
#define FDT_WIRE_INPUT_BYTES 78

/** Encoded size of A^t: throttle and cage angle. */
#define FDT_WIRE_ACT_BYTES 8

/**
 * Sentinel meaning "a camera view was attached when this input was encoded".
 *
 * Never dereferenced. A decoded fdt_input_t carries this instead of an image
 * pointer, and the RTSP path replaces it with the real frame.
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
 * Views come back as ::FDT_VIEW_PRESENT or NULL, never as an image pointer.
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
