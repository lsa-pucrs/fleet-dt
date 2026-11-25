#ifndef FLEET_DT_ASSEMBLER_H
#define FLEET_DT_ASSEMBLER_H

#include <fleet_dt/codec.h>
#include <fleet_dt/framesync.h>
#include <fleet_dt/model.h>

#include <stddef.h>
#include <stdint.h>

/**
 * @file assembler.h
 * @brief The twin side of the regulated link: rebuilds I^t per vessel from
 *        the per-sensor messages of fdt_regpub.
 *
 * Each vessel keeps one I^t. A sensor message overwrites the fields that
 * sensor measures and leaves the others as they were, so a slow sensor keeps
 * its last reading. A message older than the last one of its sensor is stale.
 * As Section V-B describes the architecture, a stale message still updates
 * the model and is only counted; ::fdt_rx_set_drop_stale switches on the
 * receiver-side drop that Section VI lists as future work.
 *
 * Frame accounting follows the two symptoms of Section V-B, per boat:
 *  - partial update: within a frame, a boat received some sensors but not
 *    all of them (::fdt_rx_incomplete), or none at all;
 *  - double update: within a frame, a sensor of a boat was applied twice,
 *    because a late message reached the input buffer beside the fresh one
 *    (::fdt_rx_doubles).
 * The first message of a new tick of a vessel also goes through the frame
 * synchroniser, when one is attached, so its per-tick counters keep their
 * meaning.
 */

/** Per-vessel state, caller-owned. */
typedef struct {
    fdt_input_t in;                       /**< The assembled I^t. */
    uint32_t    sseq[FDT_SENSOR_COUNT];   /**< Last sequence per sensor. */
    uint32_t    tick_seq;                 /**< Newest tick seen. */
    uint8_t     have;                     /**< Sensors fresh this frame. */
    uint8_t     twice;                    /**< A sensor applied twice. */
} fdt_rx_slot_t;

/** An assembler for @c n vessels. */
typedef struct {
    fdt_rx_slot_t   *slots;      /**< Caller-owned array of @c n slots. */
    size_t           n;          /**< Vessels covered. */
    fdt_framesync_t *fs;         /**< Frame synchroniser, may be NULL. */
    int              drop_stale; /**< Non-zero drops stale messages. */
    uint64_t         accepted;   /**< Sensor messages applied. */
    uint64_t         stale;      /**< Sensor messages dropped as stale. */
    uint64_t         rejected;   /**< Malformed or foreign messages. */
    uint64_t         incomplete; /**< Vessel-frames missing some sensor. */
    uint64_t         doubles;    /**< Vessel-frames with a sensor twice. */
    uint64_t         frames;     /**< Frames closed. */
    uint64_t         partial_frames; /**< Frames where some boat was not
                                          fully updated. */
} fdt_rx_t;

/** Bitmask of a vessel whose four sensors all arrived this frame. */
#define FDT_RX_HAVE_ALL ((uint8_t)((1u << FDT_SENSOR_COUNT) - 1u))

/**
 * @brief Arms an assembler.
 * @param fs  Frame synchroniser covering the same @p n vessels, or NULL.
 * @return 0 on success, -1 on a NULL argument or @p n of zero.
 */
int fdt_rx_init(fdt_rx_t *rx, fdt_rx_slot_t *slots, size_t n,
                fdt_framesync_t *fs);

/**
 * @brief Drops stale messages instead of applying them (Section VI, D6).
 */
void fdt_rx_set_drop_stale(fdt_rx_t *rx, int on);

/** @brief Opens a frame: clears the per-frame marks. */
void fdt_rx_begin_frame(fdt_rx_t *rx);

/**
 * @brief Applies one message, as delivered by a transport.
 * @return The verdict of the frame synchroniser for the first message of a
 *         tick, ::FDT_FS_FRESH for the other sensors of a tick,
 *         ::FDT_FS_STALE for a stale message (applied, or dropped when
 *         ::fdt_rx_set_drop_stale is on), or -1 when the message is
 *         malformed or carries no sensor sample.
 */
int fdt_rx_accept(fdt_rx_t *rx, const uint8_t *buf, size_t len);

/** @brief Closes a frame: counts partial and double updates. */
void fdt_rx_end_frame(fdt_rx_t *rx);

/** @brief The assembled I^t of @p vessel, or NULL. */
const fdt_input_t *fdt_rx_input(const fdt_rx_t *rx, size_t vessel);

/** @brief Sensor messages applied. */
uint64_t fdt_rx_accepted(const fdt_rx_t *rx);

/** @brief Stale sensor messages, applied or dropped. */
uint64_t fdt_rx_stale(const fdt_rx_t *rx);

/** @brief Messages refused as malformed or foreign. */
uint64_t fdt_rx_rejected(const fdt_rx_t *rx);

/** @brief Vessel-frames in which some, but not all, sensors arrived. */
uint64_t fdt_rx_incomplete(const fdt_rx_t *rx);

/** @brief Vessel-frames in which some sensor was applied twice. */
uint64_t fdt_rx_doubles(const fdt_rx_t *rx);

/** @brief Frames closed. */
uint64_t fdt_rx_frames(const fdt_rx_t *rx);

/** @brief Frames in which at least one boat missed some or all sensors. */
uint64_t fdt_rx_partial_frames(const fdt_rx_t *rx);

#endif /* FLEET_DT_ASSEMBLER_H */
