#include "fdt_rtsp.h"

#include <fleet_dt/codec.h>

#include <stddef.h>

/** Fills a buffer with a frame-dependent pattern, so frames differ. */
static void paint(uint8_t *buf, size_t len, uint32_t seq, uint8_t tint)
{
    for (size_t i = 0; i < len; i++) {
        buf[i] = (uint8_t)((seq * 31u + (uint32_t)i * 7u) ^ tint);
    }
}

static int fake_next_frame(void *self, fdt_view_t *left, fdt_view_t *right)
{
    fdt_rtsp_fake_t *f = (fdt_rtsp_fake_t *)self;
    if (f == NULL || left == NULL || right == NULL) {
        return -1;
    }

    f->seq++;
    paint(f->left, sizeof f->left, f->seq, 0x00u);
    paint(f->right, sizeof f->right, f->seq, 0xFFu);

    left->data   = f->left;
    left->len    = sizeof f->left;
    left->width  = f->width;
    left->height = f->height;
    left->seq    = f->seq;

    right->data   = f->right;
    right->len    = sizeof f->right;
    right->width  = f->width;
    right->height = f->height;
    right->seq    = f->seq;

    return 0;
}

static void fake_close(void *self)
{
    fdt_rtsp_fake_t *f = (fdt_rtsp_fake_t *)self;
    if (f != NULL) {
        f->seq = 0;
    }
}

fdt_rtsp_t fdt_rtsp_fake(fdt_rtsp_fake_t *fake, unsigned width,
                         unsigned height)
{
    fdt_rtsp_t src = { NULL, NULL, NULL };
    if (fake == NULL) {
        return src;
    }

    fake->width  = width;
    fake->height = height;
    fake->seq    = 0;

    src.next_frame = fake_next_frame;
    src.close      = fake_close;
    src.self       = fake;
    return src;
}

int fdt_rtsp_attach(fdt_input_t *in, const fdt_view_t *left,
                    const fdt_view_t *right)
{
    if (in == NULL) {
        return -1;
    }

    in->x_left  = (left != NULL && left->data != NULL)
                ? (const void *)left->data : NULL;
    in->x_right = (right != NULL && right->data != NULL)
                ? (const void *)right->data : NULL;
    return 0;
}

int fdt_rtsp_pending(const fdt_input_t *in)
{
    if (in == NULL) {
        return 0;
    }
    return (in->x_left == FDT_VIEW_PRESENT ||
            in->x_right == FDT_VIEW_PRESENT) ? 1 : 0;
}
