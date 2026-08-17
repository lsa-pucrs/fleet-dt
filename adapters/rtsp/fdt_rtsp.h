#ifndef FLEET_DT_RTSP_H
#define FLEET_DT_RTSP_H

#include <fleet_dt/model.h>

#include <stddef.h>
#include <stdint.h>

/**
 * @file fdt_rtsp.h
 * @brief The high-speed camera path, deliberately outside MQTT.
 *
 * Section III: "High-Speed Data Transfer (HSDT) comprises sensors such as
 * stereo cameras and transducers that emit larger data frames (1080p30
 * generates approximately 250 MB/s of image data). Separating the camera feed
 * from the MQTT infrastructure reduced latency while improving the DTI's
 * response time."
 *
 * The separation is the design, and this header is where it is enforced. The
 * wire codec never carries an image: fdt_enc_input() replaces each view with
 * a presence flag, and fdt_dec_input() hands back ::FDT_VIEW_PRESENT. This
 * adapter is what turns that sentinel back into a frame, on a path the
 * telemetry never touches.
 *
 * A fake source is provided so the boundary can be exercised without a camera
 * or a server. It generates frames in memory; nothing here opens a socket.
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
 *
 * Obtain one from an implementation — fdt_rtsp_fake() here, a real RTSP
 * client in deployment — and call through the members.
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
 * A decoded I^t carries ::FDT_VIEW_PRESENT where an image was, because the
 * codec never transports one. This replaces those sentinels with the real
 * frames from the high-speed path.
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
 *
 * A twin that dereferenced a sentinel would read address 1. Nothing in the
 * model does, but an application's delta^e might, so the check is available.
 */
int fdt_rtsp_pending(const fdt_input_t *in);

#endif /* FLEET_DT_RTSP_H */
