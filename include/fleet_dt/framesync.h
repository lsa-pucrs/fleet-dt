#ifndef FLEET_DT_FRAMESYNC_H
#define FLEET_DT_FRAMESYNC_H

#include <fleet_dt/envelope.h>

#include <stddef.h>
#include <stdint.h>

/**
 * @file framesync.h
 * @brief Detects and counts the frame pathology of Section V-B.
 */

/** What ::fdt_fs_accept decided about one packet. */
typedef enum {
    FDT_FS_FRESH = 0,             /**< First arrival for this vessel this frame. */
    FDT_FS_DUPLICATE_IN_FRAME = 1,/**< Newer, but this vessel already arrived. */
    FDT_FS_STALE = 2              /**< Older than what this vessel already sent. */
} fdt_fs_verdict_t;

/**
 * @brief Per-frame arrival bookkeeping for a fleet.
 */
typedef struct {
    uint32_t *seq;  /**< Highest sequence seen per vessel, caller-owned. */
    uint8_t  *hit;  /**< Arrivals this frame per vessel, caller-owned. */
    size_t    n;    /**< Vessels covered. */

    uint64_t frames;          /**< Frames closed by ::fdt_fs_end_frame. */
    uint64_t partial_frames;  /**< Frames missing at least one vessel. */
    uint64_t missing_vessels; /**< Vessel-frames that contributed nothing. */
    uint64_t double_updates;  /**< Second-and-later arrivals within a frame. */
    uint64_t stale_packets;   /**< Packets older than the vessel's high water. */
    uint64_t accepted;        /**< Packets judged ::FDT_FS_FRESH. */
} fdt_framesync_t;

/**
 * @brief Binds the monitor to caller-owned per-vessel storage.
 * @param seq_slots  Array of @p n sequence high-water marks.
 * @param hit_slots  Array of @p n per-frame arrival flags.
 * @return 0 on success, -1 when an argument is NULL or @p n is zero.
 */
int fdt_fs_init(fdt_framesync_t *fs, uint32_t *seq_slots, uint8_t *hit_slots,
                size_t n);

/** @brief Opens a frame, clearing the per-vessel arrival flags. */
void fdt_fs_begin_frame(fdt_framesync_t *fs);

/**
 * @brief Judges one arriving packet against this frame and this vessel.
 * @return The verdict. An out-of-range vessel index yields ::FDT_FS_STALE
 *         without touching any counter, since it belongs to no vessel this
 *         monitor covers.
 */
fdt_fs_verdict_t fdt_fs_accept(fdt_framesync_t *fs, const fdt_env_t *env);

/**
 * @brief Closes a frame and folds it into the partial-frame statistics.
 */
void fdt_fs_end_frame(fdt_framesync_t *fs);

/** @brief Frames closed so far. */
uint64_t fdt_fs_frames(const fdt_framesync_t *fs);

/** @brief Frames in which at least one vessel contributed nothing. */
uint64_t fdt_fs_partial_frames(const fdt_framesync_t *fs);

/** @brief Vessel-frames that contributed nothing, summed over all frames. */
uint64_t fdt_fs_missing_vessels(const fdt_framesync_t *fs);

/** @brief Second-and-later arrivals for one vessel within a single frame. */
uint64_t fdt_fs_double_updates(const fdt_framesync_t *fs);

/** @brief Packets older than the sequence already seen for that vessel. */
uint64_t fdt_fs_stale_packets(const fdt_framesync_t *fs);

/** @brief Packets judged fresh. */
uint64_t fdt_fs_accepted(const fdt_framesync_t *fs);

#endif /* FLEET_DT_FRAMESYNC_H */
