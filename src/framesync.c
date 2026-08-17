#include <fleet_dt/framesync.h>

#include <string.h>

int fdt_fs_init(fdt_framesync_t *fs, uint32_t *seq_slots, uint8_t *hit_slots,
                size_t n)
{
    if (fs == NULL || seq_slots == NULL || hit_slots == NULL || n == 0) {
        return -1;
    }

    memset(fs, 0, sizeof *fs);
    fs->seq = seq_slots;
    fs->hit = hit_slots;
    fs->n   = n;

    for (size_t k = 0; k < n; k++) {
        fs->seq[k] = 0;
        fs->hit[k] = 0;
    }
    return 0;
}

void fdt_fs_begin_frame(fdt_framesync_t *fs)
{
    if (fs == NULL || fs->hit == NULL) {
        return;
    }
    for (size_t k = 0; k < fs->n; k++) {
        fs->hit[k] = 0;
    }
}

fdt_fs_verdict_t fdt_fs_accept(fdt_framesync_t *fs, const fdt_env_t *env)
{
    if (fs == NULL || fs->seq == NULL || env == NULL) {
        return FDT_FS_STALE;
    }
    if ((size_t)env->vessel >= fs->n) {
        return FDT_FS_STALE;
    }

    const size_t k = (size_t)env->vessel;

    if (env->seq <= fs->seq[k]) {
        fs->stale_packets++;
        return FDT_FS_STALE;
    }

    fs->seq[k] = env->seq;

    if (fs->hit[k] != 0) {
        fs->double_updates++;
        return FDT_FS_DUPLICATE_IN_FRAME;
    }

    fs->hit[k] = 1;
    fs->accepted++;
    return FDT_FS_FRESH;
}

void fdt_fs_end_frame(fdt_framesync_t *fs)
{
    if (fs == NULL || fs->hit == NULL) {
        return;
    }

    fs->frames++;

    uint64_t missing = 0;
    for (size_t k = 0; k < fs->n; k++) {
        if (fs->hit[k] == 0) {
            missing++;
        }
    }
    if (missing > 0) {
        fs->partial_frames++;
        fs->missing_vessels += missing;
    }
}

uint64_t fdt_fs_frames(const fdt_framesync_t *fs)
{
    return (fs == NULL) ? 0 : fs->frames;
}

uint64_t fdt_fs_partial_frames(const fdt_framesync_t *fs)
{
    return (fs == NULL) ? 0 : fs->partial_frames;
}

uint64_t fdt_fs_missing_vessels(const fdt_framesync_t *fs)
{
    return (fs == NULL) ? 0 : fs->missing_vessels;
}

uint64_t fdt_fs_double_updates(const fdt_framesync_t *fs)
{
    return (fs == NULL) ? 0 : fs->double_updates;
}

uint64_t fdt_fs_stale_packets(const fdt_framesync_t *fs)
{
    return (fs == NULL) ? 0 : fs->stale_packets;
}

uint64_t fdt_fs_accepted(const fdt_framesync_t *fs)
{
    return (fs == NULL) ? 0 : fs->accepted;
}
