/**
 * @file test_wire.c
 * @brief Codec, envelope, and the partial/double update detector.
 */
#include <fleet_dt/codec.h>
#include <fleet_dt/envelope.h>
#include <fleet_dt/framesync.h>

#include <assert.h>
#include <stdio.h>
#include <string.h>

/** The three encoded widths, and why the state's matches its in-memory size. */
static void test_wire_widths(void)
{
    assert(FDT_WIRE_STATE_BYTES == 48);
    assert(FDT_WIRE_INPUT_BYTES == 78);
    assert(FDT_WIRE_ACT_BYTES == 8);

    assert((size_t)FDT_WIRE_STATE_BYTES == sizeof(fdt_state_t));

    /* 19 scalars plus two presence flags. */
    assert(FDT_WIRE_INPUT_BYTES == 19 * 4 + 2);
}

/** The byte order is a property of the protocol, not of the host. */
static void test_explicit_little_endian(void)
{
    const fdt_state_t one = { .lat_deg = 1.0f };
    uint8_t buf[FDT_WIRE_STATE_BYTES];

    assert(fdt_enc_state(&one, buf, sizeof buf) == FDT_WIRE_STATE_BYTES);
    assert(buf[0] == 0x00 && buf[1] == 0x00 &&
           buf[2] == 0x80 && buf[3] == 0x3F);

    /* -2.0f is 0xC0000000: 00 00 00 C0. */
    const fdt_state_t neg = { .lat_deg = -2.0f };
    assert(fdt_enc_state(&neg, buf, sizeof buf) == FDT_WIRE_STATE_BYTES);
    assert(buf[0] == 0x00 && buf[1] == 0x00 &&
           buf[2] == 0x00 && buf[3] == 0xC0);
}

/** Round-trips, and the refusal to work in a short buffer. */
static void test_codec_round_trips(void)
{
    uint8_t buf[128];

    const fdt_state_t b = {
        .lat_deg = -30.05f, .lon_deg = -51.17f, .alt_m = 12.5f,
        .roll_deg = 1.5f, .pitch_deg = -2.5f, .yaw_deg = 91.25f,
        .surge_mps = 1.25f, .sway_mps = -0.5f, .heave_mps = 0.0f,
        .roll_rate_rps = 0.1f, .pitch_rate_rps = -0.2f, .yaw_rate_rps = 0.3f,
    };

    assert(fdt_enc_state(&b, buf, sizeof buf) == FDT_WIRE_STATE_BYTES);
    assert(fdt_enc_state(&b, buf, 4) == -1);
    assert(fdt_enc_state(NULL, buf, sizeof buf) == -1);

    fdt_state_t b_out = {0};
    assert(fdt_dec_state(buf, FDT_WIRE_STATE_BYTES, &b_out) ==
           FDT_WIRE_STATE_BYTES);
    assert(fdt_dec_state(buf, 4, &b_out) == -1);
    assert(memcmp(&b, &b_out, sizeof b) == 0);

    fdt_input_t in = {0};
    in.ax_mps2 = 1.0f;
    in.ibat_a  = 9.5f;
    in.x_left  = buf;   /* a real pointer on this side */
    in.x_right = NULL;

    assert(fdt_enc_input(&in, buf, sizeof buf) == FDT_WIRE_INPUT_BYTES);
    fdt_input_t in_out = {0};
    assert(fdt_dec_input(buf, FDT_WIRE_INPUT_BYTES, &in_out) ==
           FDT_WIRE_INPUT_BYTES);
    assert(in_out.ax_mps2 == 1.0f && in_out.ibat_a == 9.5f);

    assert(in_out.x_left == FDT_VIEW_PRESENT);
    assert(in_out.x_right == NULL);

    const fdt_actuation_t a = { .throttle_pct = 60.0f, .cage_rad = -0.25f };
    assert(fdt_enc_act(&a, buf, sizeof buf) == FDT_WIRE_ACT_BYTES);
    fdt_actuation_t a_out = {0};
    assert(fdt_dec_act(buf, FDT_WIRE_ACT_BYTES, &a_out) == FDT_WIRE_ACT_BYTES);
    assert(memcmp(&a, &a_out, sizeof a) == 0);

    printf("wire widths: state=%d input=%d act=%d bytes\n",
           FDT_WIRE_STATE_BYTES, FDT_WIRE_INPUT_BYTES, FDT_WIRE_ACT_BYTES);
}

/** The envelope carries the sequence number Table I does not. */
static void test_envelope(void)
{
    assert(FDT_ENV_HEADER_BYTES == 16);

    const fdt_state_t b = { .yaw_deg = 42.0f };
    uint8_t payload[FDT_WIRE_STATE_BYTES];
    assert(fdt_enc_state(&b, payload, sizeof payload) == FDT_WIRE_STATE_BYTES);

    const fdt_env_t env = { .magic = FDT_ENV_MAGIC, .vessel = 3,
                            .kind = FDT_ENV_STATE, .flags = 0, .seq = 1000,
                            .payload_len = FDT_WIRE_STATE_BYTES };
    uint8_t frame[128];

    const long nw = fdt_env_encode(&env, payload, sizeof payload,
                                   frame, sizeof frame);
    assert(nw == FDT_ENV_HEADER_BYTES + FDT_WIRE_STATE_BYTES);
    assert(fdt_env_encode(&env, payload, sizeof payload, frame, 8) == -1);

    assert(fdt_env_encode(&env, payload, 4, frame, sizeof frame) == -1);

    fdt_env_t got = {0};
    const uint8_t *pl = NULL;
    assert(fdt_env_decode(frame, (size_t)nw, &got, &pl) == nw);
    assert(got.magic == FDT_ENV_MAGIC && got.vessel == 3);
    assert(got.kind == FDT_ENV_STATE && got.seq == 1000);
    assert(pl != NULL);

    fdt_state_t rt = {0};
    assert(fdt_dec_state(pl, got.payload_len, &rt) == FDT_WIRE_STATE_BYTES);
    assert(rt.yaw_deg == 42.0f);

    /* Corrupt magic, unknown kind, and truncation are all rejected whole. */
    frame[0] ^= 0xFF;
    assert(fdt_env_decode(frame, (size_t)nw, &got, &pl) == -1);
    frame[0] ^= 0xFF;

    frame[6] = 99;
    assert(fdt_env_decode(frame, (size_t)nw, &got, &pl) == -1);
    frame[6] = FDT_ENV_STATE;

    assert(fdt_env_decode(frame, FDT_ENV_HEADER_BYTES + 4, &got, &pl) == -1);
    assert(fdt_env_decode(frame, 4, &got, &pl) == -1);

    /* The header alone is readable without asking for the payload. */
    assert(fdt_env_decode(frame, (size_t)nw, &got, NULL) == nw);
}

/** Builds a state envelope for one vessel at one sequence. */
static fdt_env_t state_env(uint16_t vessel, uint32_t seq)
{
    fdt_env_t e = {0};
    e.magic  = FDT_ENV_MAGIC;
    e.vessel = vessel;
    e.kind   = FDT_ENV_STATE;
    e.seq    = seq;
    return e;
}

/** C27: the three failures of Section V-B, counted apart. */
static void test_framesync(void)
{
    uint32_t seqs[3];
    uint8_t  hits[3];
    fdt_framesync_t fs;

    assert(fdt_fs_init(&fs, seqs, hits, 3) == 0);
    assert(fdt_fs_init(&fs, NULL, hits, 3) == -1);
    assert(fdt_fs_init(&fs, seqs, hits, 0) == -1);
    assert(fdt_fs_init(&fs, seqs, hits, 3) == 0);

    /* A healthy frame: all three vessels arrive once. */
    fdt_fs_begin_frame(&fs);
    for (uint16_t k = 0; k < 3; k++) {
        const fdt_env_t e = state_env(k, 1);
        assert(fdt_fs_accept(&fs, &e) == FDT_FS_FRESH);
    }
    fdt_fs_end_frame(&fs);
    assert(fdt_fs_frames(&fs) == 1);
    assert(fdt_fs_partial_frames(&fs) == 0);
    assert(fdt_fs_double_updates(&fs) == 0);
    assert(fdt_fs_accepted(&fs) == 3);

    fdt_fs_begin_frame(&fs);
    const fdt_env_t v0a = state_env(0, 2);
    const fdt_env_t v0b = state_env(0, 3);
    const fdt_env_t v1  = state_env(1, 2);
    assert(fdt_fs_accept(&fs, &v0a) == FDT_FS_FRESH);
    assert(fdt_fs_accept(&fs, &v0b) == FDT_FS_DUPLICATE_IN_FRAME);
    assert(fdt_fs_accept(&fs, &v1) == FDT_FS_FRESH);
    fdt_fs_end_frame(&fs);

    assert(fdt_fs_double_updates(&fs) == 1);
    assert(fdt_fs_partial_frames(&fs) == 1);
    assert(fdt_fs_missing_vessels(&fs) == 1);

    fdt_fs_begin_frame(&fs);
    const fdt_env_t old = state_env(0, 1);
    assert(fdt_fs_accept(&fs, &old) == FDT_FS_STALE);
    fdt_fs_end_frame(&fs);
    assert(fdt_fs_stale_packets(&fs) == 1);

    assert(fdt_fs_accepted(&fs) == 5);

    /* A vessel outside the fleet touches no counter. */
    const uint64_t stale_before = fdt_fs_stale_packets(&fs);
    const fdt_env_t stranger = state_env(9, 5);
    assert(fdt_fs_accept(&fs, &stranger) == FDT_FS_STALE);
    assert(fdt_fs_stale_packets(&fs) == stale_before);

    printf("framesync: frames=%llu partial=%llu double=%llu stale=%llu\n",
           (unsigned long long)fdt_fs_frames(&fs),
           (unsigned long long)fdt_fs_partial_frames(&fs),
           (unsigned long long)fdt_fs_double_updates(&fs),
           (unsigned long long)fdt_fs_stale_packets(&fs));
}

int main(void)
{
    test_wire_widths();
    test_explicit_little_endian();
    test_codec_round_trips();
    test_envelope();
    test_framesync();

    printf("test_wire: ok\n");
    return 0;
}
