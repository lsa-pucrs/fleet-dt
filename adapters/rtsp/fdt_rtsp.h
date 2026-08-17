#ifndef FLEET_DT_RTSP_H
#define FLEET_DT_RTSP_H

#include <fleet_dt/model.h>

#include <stddef.h>
#include <stdint.h>

/**
 * @file fdt_rtsp.h
 * @brief The high-speed camera path, deliberately outside MQTT.
 */

/** One camera frame. Memory belongs to the source, not to the caller. */
typedef struct {
    const uint8_t *data;   /**< Frame bytes; NULL when no frame is ready. */
    size_t         len;    /**< Bytes in @c data. */
    unsigned       width;  /**< Frame width in pixels. */
    unsigned       height; /**< Frame height in pixels. */
    uint32_t       seq;    /**< Monotonic frame counter. */
} fdt_view_t;

/**
 * @brief A stereo camera source.
 */
typedef struct {
    /**
     * Fetches the next stereo pair.
     * @return 0 when both views were filled, -1 when no frame is ready.
     */
    int (*next_frame)(void *self, fdt_view_t *left, fdt_view_t *right);

    /** Releases whatever the implementation holds. */
    void (*close)(void *self);

    /** Implementation state. */
    void *self;
} fdt_rtsp_t;

/** Frame size the fake source generates, in bytes per view. */
#define FDT_FAKE_FRAME_BYTES 64

/** State of the in-memory fake source. */
typedef struct {
    unsigned width;
    unsigned height;
    uint32_t seq;
    uint8_t  left[FDT_FAKE_FRAME_BYTES];
    uint8_t  right[FDT_FAKE_FRAME_BYTES];
} fdt_rtsp_fake_t;

/**
 * @brief Builds a camera source that fabricates frames in memory.
 * @param width,height  Reported frame geometry.
 * @return A source whose @c self points at @p fake, which must outlive it.
 *         On a NULL argument every member is NULL.
 */
fdt_rtsp_t fdt_rtsp_fake(fdt_rtsp_fake_t *fake, unsigned width,
                         unsigned height);

/**
 * @brief Re-attaches camera views to an input that came off the wire.
 *
 * @param in     The input to complete; its view fields are overwritten.
 * @param left   Left view, or NULL to clear the field.
 * @param right  Right view, or NULL to clear the field.
 * @return 0 on success, -1 when @p in is NULL.
 *
 * @note Views whose data pointer is NULL clear the corresponding field rather
 *       than leaving the sentinel in place, so a downstream reader can never
 *       mistake "a frame was attached upstream" for "a frame is available
 *       here".
 */
int fdt_rtsp_attach(fdt_input_t *in, const fdt_view_t *left,
                    const fdt_view_t *right);

/**
 * @brief Whether an input still carries unresolved view sentinels.
 * @return 1 when either field is ::FDT_VIEW_PRESENT, 0 otherwise.
 */
int fdt_rtsp_pending(const fdt_input_t *in);

#endif /* FLEET_DT_RTSP_H */
