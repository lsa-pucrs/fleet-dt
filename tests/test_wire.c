/**
 * @file test_wire.c
 * @brief Codec, envelope, and the partial/double update detector.
 */
#include <fleet_dt/assembler.h>
#include <fleet_dt/codec.h>
#include <fleet_dt/envelope.h>
#include <fleet_dt/framesync.h>

#include <assert.h>
#include <stddef.h>
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

/** One message per sensor: the four codecs cover every scalar of I^t. */
static void test_sensor_codec(void)
{
    const fdt_input_t in = {
        .ax_mps2 = 0.1f, .ay_mps2 = 0.2f, .az_mps2 = -9.8f,
        .wx_rps = 0.01f, .wy_rps = 0.02f, .wz_rps = 0.15f,
        .mx_ut = 18.0f, .my_ut = 2.0f, .mz_ut = -14.0f,
        .gps_lat_deg = -30.05f, .gps_lon_deg = -51.17f, .gps_alt_m = 3.0f,
        .vn_mps = 1.2f, .ve_mps = 0.4f, .vd_mps = 0.01f,
        .press_pa = 101325.0f, .temp_c = 22.0f,
        .vbat_v = 18.5f, .ibat_a = 6.0f,
    };

    size_t total = 0;
    fdt_input_t out = {0};
    for (int s = 0; s < FDT_SENSOR_COUNT; s++) {
        uint8_t buf[FDT_WIRE_SENSOR_MAX_BYTES];
        const long n = fdt_enc_sensor(&in, (fdt_sensor_t)s, buf, sizeof buf);
        assert(n == (long)fdt_sensor_bytes((fdt_sensor_t)s));
        assert(fdt_enc_sensor(&in, (fdt_sensor_t)s, buf, (size_t)n - 1) == -1);
        assert(fdt_dec_sensor(buf, (size_t)n - 1, (fdt_sensor_t)s, &out) == -1);
        assert(fdt_dec_sensor(buf, (size_t)n, (fdt_sensor_t)s, &out) == n);
        assert(fdt_sensor_name((fdt_sensor_t)s) != NULL);
        assert(fdt_env_kind_sensor(fdt_env_sensor_kind(s)) == s);
        total += (size_t)n;
    }
    /* 19 floats: every scalar of Table I except the two stereo views. */
    assert(total == 19 * 4);
    assert(memcmp(&in, &out, offsetof(fdt_input_t, x_left)) == 0);

    assert(fdt_sensor_bytes(FDT_SENSOR_COUNT) == 0);
    assert(fdt_sensor_name(FDT_SENSOR_COUNT) == NULL);
    assert(fdt_env_sensor_kind(FDT_SENSOR_COUNT) == 0);
    assert(fdt_env_kind_sensor(FDT_ENV_STATE) == -1);

    /* A barometer sample leaves the IMU fields alone. */
    fdt_input_t partial = { .wz_rps = 7.0f };
    uint8_t buf[FDT_WIRE_BARO_BYTES];
    assert(fdt_enc_sensor(&in, FDT_SENSOR_BARO, buf, sizeof buf) == 8);
    assert(fdt_dec_sensor(buf, sizeof buf, FDT_SENSOR_BARO, &partial) == 8);
    assert(partial.wz_rps == 7.0f && partial.press_pa == 101325.0f);
}

/** Wraps one sensor sample of @p vessel at sequence @p seq. */
static size_t sensor_frame(uint8_t *out, size_t cap, uint16_t vessel,
                           fdt_sensor_t s, uint32_t seq, float value)
{
    fdt_input_t in = {0};
    in.wz_rps = value;
    in.gps_lat_deg = value;
    in.press_pa = value;
    in.vbat_v = value;
    uint8_t payload[FDT_WIRE_SENSOR_MAX_BYTES];
    const long n = fdt_enc_sensor(&in, s, payload, sizeof payload);
    assert(n > 0);
    const fdt_env_t env = { .magic = FDT_ENV_MAGIC, .vessel = vessel,
                            .kind = fdt_env_sensor_kind((int)s),
                            .flags = 0, .seq = seq,
                            .payload_len = (uint32_t)n };
    const long m = fdt_env_encode(&env, payload, (size_t)n, out, cap);
    assert(m > 0);
    return (size_t)m;
}

/** The twin side: stale drops, boat-frame accounting, incomplete frames. */
static void test_assembler(void)
{
    fdt_rx_slot_t   slots[2];
    uint32_t        seqs[2];
    uint8_t         hits[2];
    fdt_framesync_t fs;
    fdt_rx_t        rx;
    uint8_t         f[64];

    assert(fdt_rx_init(NULL, slots, 2, &fs) == -1);
    assert(fdt_rx_init(&rx, slots, 0, &fs) == -1);
    assert(fdt_fs_init(&fs, seqs, hits, 2) == 0);
    assert(fdt_rx_init(&rx, slots, 2, &fs) == 0);

    /* Frame 1: vessel 0 sends all four sensors of tick 1; vessel 1 sends
       only its IMU. */
    fdt_rx_begin_frame(&rx);
    for (int s = 0; s < FDT_SENSOR_COUNT; s++) {
        const size_t n = sensor_frame(f, sizeof f, 0, (fdt_sensor_t)s, 1,
                                      1.0f);
        assert(fdt_rx_accept(&rx, f, n) == FDT_FS_FRESH);
    }
    size_t n = sensor_frame(f, sizeof f, 1, FDT_SENSOR_IMU, 1, 1.0f);
    assert(fdt_rx_accept(&rx, f, n) == FDT_FS_FRESH);
    fdt_rx_end_frame(&rx);
    assert(fdt_fs_accepted(&fs) == 2);   /* two boat-frames, not five */
    assert(fdt_rx_incomplete(&rx) == 1); /* vessel 1 lacked 3 sensors */
    assert(fdt_rx_partial_frames(&rx) == 1);
    assert(fdt_fs_partial_frames(&fs) == 0);

    /* Frame 2: vessel 0 delivers tick 2, then a late tick 1 IMU (stale),
       then tick 3 in the same frame (a double update). Vessel 1 is silent. */
    fdt_rx_begin_frame(&rx);
    n = sensor_frame(f, sizeof f, 0, FDT_SENSOR_IMU, 2, 2.0f);
    assert(fdt_rx_accept(&rx, f, n) == FDT_FS_FRESH);
    /* Stale, and applied: Section V-B's architecture has no drop. */
    n = sensor_frame(f, sizeof f, 0, FDT_SENSOR_IMU, 1, 9.0f);
    assert(fdt_rx_accept(&rx, f, n) == FDT_FS_STALE);
    assert(fdt_rx_input(&rx, 0)->wz_rps == 9.0f);
    n = sensor_frame(f, sizeof f, 0, FDT_SENSOR_IMU, 3, 3.0f);
    assert(fdt_rx_accept(&rx, f, n) == FDT_FS_DUPLICATE_IN_FRAME);
    assert(fdt_rx_input(&rx, 0)->wz_rps == 3.0f);
    fdt_rx_end_frame(&rx);
    assert(fdt_rx_stale(&rx) == 1);
    assert(fdt_rx_doubles(&rx) == 1);    /* the IMU of vessel 0, thrice */
    assert(fdt_rx_frames(&rx) == 2);
    assert(fdt_fs_double_updates(&fs) == 1);
    assert(fdt_fs_partial_frames(&fs) == 1);
    /* The GPS of vessel 0 kept its tick 1 reading. */
    assert(fdt_rx_input(&rx, 0)->gps_lat_deg == 1.0f);

    /* With the Section VI drop on, a stale message leaves the model alone. */
    fdt_rx_set_drop_stale(&rx, 1);
    n = sensor_frame(f, sizeof f, 0, FDT_SENSOR_IMU, 2, 5.0f);
    assert(fdt_rx_accept(&rx, f, n) == FDT_FS_STALE);
    assert(fdt_rx_input(&rx, 0)->wz_rps == 3.0f);
    assert(fdt_rx_stale(&rx) == 2);
    fdt_rx_set_drop_stale(&rx, 0);

    /* Malformed, foreign and out-of-range messages are refused. */
    uint8_t junk[4] = { 1, 2, 3, 4 };
    assert(fdt_rx_accept(&rx, junk, sizeof junk) == -1);
    n = sensor_frame(f, sizeof f, 7, FDT_SENSOR_IMU, 1, 1.0f);
    assert(fdt_rx_accept(&rx, f, n) == -1);
    assert(fdt_rx_rejected(&rx) == 2);
    assert(fdt_rx_accept(NULL, f, n) == -1);
    assert(fdt_rx_input(&rx, 2) == NULL);
}

int main(void)
{
    test_wire_widths();
    test_sensor_codec();
    test_assembler();
    test_explicit_little_endian();
    test_codec_round_trips();
    test_envelope();
    test_framesync();

    printf("test_wire: ok\n");
    return 0;
}
