#ifndef FLEET_DT_ENVELOPE_H
#define FLEET_DT_ENVELOPE_H

#include <stddef.h>
#include <stdint.h>

/**
 * @file envelope.h
 * @brief The wire envelope, carrying what Table I does not.
 */

/** Magic word "FDT1", identifying the envelope format. */
#define FDT_ENV_MAGIC 0x46445431u

/** Header size in bytes, ahead of the payload. */
#define FDT_ENV_HEADER_BYTES 16

/** What the payload holds. */
typedef enum {
    FDT_ENV_INPUT = 1, /**< I^t, encoded by fdt_enc_input(). */
    FDT_ENV_STATE = 2, /**< B^t, encoded by fdt_enc_state(). */
    FDT_ENV_ACT   = 3, /**< A^t, encoded by fdt_enc_act(). */
    FDT_ENV_GOAL  = 4  /**< g^t, encoded by fdt_enc_state(). */
} fdt_env_kind_t;

/** The envelope header. */
typedef struct {
    uint32_t magic;       /**< ::FDT_ENV_MAGIC. */
    uint16_t vessel;      /**< Vessel index this message concerns. */
    uint8_t  kind;        /**< One of ::fdt_env_kind_t. */
    uint8_t  flags;       /**< Reserved; must be zero. */
    uint32_t seq;         /**< Per-vessel, per-kind monotonic counter. */
    uint32_t payload_len; /**< Payload bytes following the header. */
} fdt_env_t;

/**
 * @brief Writes a header followed by its payload.
 * @param env      Header to write; @c payload_len must equal @p plen.
 * @param payload  Payload bytes, may be NULL when @p plen is zero.
 * @param plen     Payload length.
 * @param buf      Destination.
 * @param cap      Capacity of @p buf.
 * @return Total bytes written, or -1 when an argument is inconsistent or
 *         @p cap is too small.
 */
long fdt_env_encode(const fdt_env_t *env, const uint8_t *payload, size_t plen,
                    uint8_t *buf, size_t cap);

/**
 * @brief Reads a header and locates its payload.
 * @param buf          Frame bytes.
 * @param len          Bytes available.
 * @param env          Receives the decoded header.
 * @param payload_out  Receives a pointer into @p buf, not a copy. May be NULL
 *                     if the caller only wants the header.
 * @return Total bytes consumed, or -1 when the magic is wrong, the kind is
 *         unknown, or the frame is truncated.
 */
long fdt_env_decode(const uint8_t *buf, size_t len, fdt_env_t *env,
                    const uint8_t **payload_out);

#endif /* FLEET_DT_ENVELOPE_H */
