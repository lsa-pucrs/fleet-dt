#include <fleet_dt/assembler.h>

#include <fleet_dt/envelope.h>

int fdt_rx_init(fdt_rx_t *rx, fdt_rx_slot_t *slots, size_t n,
                fdt_framesync_t *fs)
{
    if (rx == NULL || slots == NULL || n == 0) {
        return -1;
    }

    rx->slots      = slots;
    rx->n          = n;
    rx->fs         = fs;
    rx->drop_stale = 0;
    rx->accepted   = 0;
    rx->stale      = 0;
    rx->rejected   = 0;
    rx->incomplete = 0;
    rx->doubles    = 0;
    rx->frames     = 0;
    rx->partial_frames = 0;
    for (size_t k = 0; k < n; k++) {
        slots[k] = (fdt_rx_slot_t){0};
    }
    return 0;
}

void fdt_rx_set_drop_stale(fdt_rx_t *rx, int on)
{
    if (rx != NULL) {
        rx->drop_stale = on;
    }
}

void fdt_rx_begin_frame(fdt_rx_t *rx)
{
    if (rx == NULL) {
        return;
    }
    for (size_t k = 0; k < rx->n; k++) {
        rx->slots[k].have  = 0;
        rx->slots[k].twice = 0;
    }
    if (rx->fs != NULL) {
        fdt_fs_begin_frame(rx->fs);
    }
}

int fdt_rx_accept(fdt_rx_t *rx, const uint8_t *buf, size_t len)
{
    if (rx == NULL) {
        return -1;
    }

    fdt_env_t      env;
    const uint8_t *payload = NULL;
    if (buf == NULL || fdt_env_decode(buf, len, &env, &payload) < 0) {
        rx->rejected++;
        return -1;
    }

    const int s = fdt_env_kind_sensor(env.kind);
    if (s < 0 || (size_t)env.vessel >= rx->n || payload == NULL) {
        rx->rejected++;
        return -1;
    }

    fdt_rx_slot_t *slot = &rx->slots[env.vessel];
    int verdict = FDT_FS_FRESH;
    const int stale = (env.seq <= slot->sseq[s]);
    if (stale) {
        rx->stale++;
        verdict = FDT_FS_STALE;
        if (rx->drop_stale) {
            return FDT_FS_STALE;
        }
    } else if (env.seq > slot->tick_seq) {
        /* First message of a new tick of this vessel: one boat-frame. */
        if (rx->fs != NULL) {
            verdict = (int)fdt_fs_accept(rx->fs, &env);
        }
        slot->tick_seq = env.seq;
    }

    if (fdt_dec_sensor(payload, env.payload_len, (fdt_sensor_t)s,
                       &slot->in) < 0) {
        rx->rejected++;
        return -1;
    }
    if (!stale) {
        slot->sseq[s] = env.seq;
    }
    if (slot->have & (uint8_t)(1u << s)) {
        slot->twice = 1;
    }
    slot->have   |= (uint8_t)(1u << s);
    rx->accepted++;
    return verdict;
}

void fdt_rx_end_frame(fdt_rx_t *rx)
{
    if (rx == NULL) {
        return;
    }
    int partial = 0;
    for (size_t k = 0; k < rx->n; k++) {
        const uint8_t have = rx->slots[k].have;
        if (have != 0 && have != FDT_RX_HAVE_ALL) {
            rx->incomplete++;
        }
        if (have != FDT_RX_HAVE_ALL) {
            partial = 1;
        }
        if (rx->slots[k].twice) {
            rx->doubles++;
        }
    }
    rx->frames++;
    if (partial) {
        rx->partial_frames++;
    }
    if (rx->fs != NULL) {
        fdt_fs_end_frame(rx->fs);
    }
}

const fdt_input_t *fdt_rx_input(const fdt_rx_t *rx, size_t vessel)
{
    if (rx == NULL || vessel >= rx->n) {
        return NULL;
    }
    return &rx->slots[vessel].in;
}

uint64_t fdt_rx_accepted(const fdt_rx_t *rx)
{
    return (rx == NULL) ? 0 : rx->accepted;
}

uint64_t fdt_rx_stale(const fdt_rx_t *rx)
{
    return (rx == NULL) ? 0 : rx->stale;
}

uint64_t fdt_rx_rejected(const fdt_rx_t *rx)
{
    return (rx == NULL) ? 0 : rx->rejected;
}

uint64_t fdt_rx_incomplete(const fdt_rx_t *rx)
{
    return (rx == NULL) ? 0 : rx->incomplete;
}

uint64_t fdt_rx_doubles(const fdt_rx_t *rx)
{
    return (rx == NULL) ? 0 : rx->doubles;
}

uint64_t fdt_rx_frames(const fdt_rx_t *rx)
{
    return (rx == NULL) ? 0 : rx->frames;
}

uint64_t fdt_rx_partial_frames(const fdt_rx_t *rx)
{
    return (rx == NULL) ? 0 : rx->partial_frames;
}
