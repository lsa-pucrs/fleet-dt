/**
 * @file test_mavlink_wire.c
 * @brief MAVLink framing against frames pymavlink 2.4 produced (common.xml,
 *        sysid 1, compid 1), so the bytes come from an implementation other
 *        than the one under test. The vectors are data; nothing here links
 *        pymavlink.
 */
#define _POSIX_C_SOURCE 200809L

#include "mavlink/fdt_mavlink_udp.h"
#include "mavlink/fdt_mavlink_wire.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Telemetry Ardupilot sends, Table I. SCALED_IMU2 1000 mg ahead and -1000 mg
 * down, 500/-250 mrad/s, 230/-140 mG; GPS -30.05, -51.17, 3 m; NED velocity
 * 1.25, -0.5, 0.02 m/s; 1013.25 hPa at 22.50 C; a pack of 18.5 V, 6.2 A. */
static const uint8_t IMU[] = { 0xfd, 0x16, 0x00, 0x00, 0x0a, 0x01, 0x01, 0x74, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0x18, 0xfc, 0xf4, 0x01, 0x06, 0xff, 0x00, 0x00, 0xe6, 0x00, 0x00, 0x00, 0x74, 0xff, 0x4f, 0x81 };
static const uint8_t GPS[] = { 0xfd, 0x1e, 0x00, 0x00, 0x0b, 0x01, 0x01, 0x18, 0x00, 0x00, 0x7b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xbb, 0x16, 0xee, 0xe0, 0x13, 0x80, 0xe1, 0xb8, 0x0b, 0x00, 0x00, 0x64, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0a, 0x00, 0x45 };
static const uint8_t NED[] = { 0xfd, 0x1c, 0x00, 0x00, 0x0c, 0x01, 0x01, 0x20, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x3f, 0x00, 0x00, 0x00, 0xbf, 0x0a, 0xd7, 0xa3, 0x3c, 0xab, 0xdf };
static const uint8_t PRESS[] = { 0xfd, 0x0e, 0x00, 0x00, 0x0d, 0x01, 0x01, 0x1d, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0x00, 0x50, 0x7d, 0x44, 0x00, 0x00, 0x00, 0x00, 0xca, 0x08, 0x32, 0x89 };
static const uint8_t BAT[] = { 0xfd, 0x24, 0x00, 0x00, 0x0e, 0x01, 0x01, 0x93, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc4, 0x09, 0x44, 0x48, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x6c, 0x02, 0x00, 0x01, 0x01, 0x50, 0x38, 0x1d };
static const uint8_t PRESS_SIGNED[] = { 0xfd, 0x0e, 0x01, 0x00, 0x00, 0x01, 0x01, 0x1d, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0x00, 0x50, 0x7d, 0x44, 0x00, 0x00, 0x00, 0x00, 0xca, 0x08, 0x56, 0x59, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5c, 0x17, 0xbb, 0xbe, 0x9e, 0x40 };
static const uint8_t HB_V1[] = { 0xfe, 0x09, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x00, 0x04, 0x03, 0xad, 0x18 };

static int feed(fdt_mav_parser_t *p, const uint8_t *b, size_t n,
                fdt_mav_msg_t *out)
{
    int got = 0;
    for (size_t i = 0; i < n; i++) {
        got += fdt_mav_parse(p, b[i], out);
    }
    return got;
}

static void test_crc(void)
{
    /* CRC-16/MCRF4XX check value of "123456789". */
    assert(fdt_mav_crc(0xFFFFu, (const uint8_t *)"123456789", 9) == 0x6F91u);
    assert(fdt_mav_crc_extra(FDT_MAVMSG_HEARTBEAT) == 50);
    assert(fdt_mav_crc_extra(FDT_MAVMSG_SCALED_IMU2) == 76);
    assert(fdt_mav_crc_extra(9999) == -1);
}

static void test_telemetry_stream(void)
{
    fdt_mav_parser_t p;
    memset(&p, 0, sizeof p);
    fdt_mav_ingest_t ing;
    fdt_mav_init(&ing);
    fdt_mav_msg_t m;

    /* Noise before the first frame is skipped. */
    const uint8_t noise[] = { 0x00, 0x13, 0x37, 0xFD, 0x02 };
    assert(feed(&p, noise, 3, &m) == 0);

    const struct { const uint8_t *b; size_t n; uint32_t id; } F[] = {
        { IMU, sizeof IMU, FDT_MAVMSG_SCALED_IMU2 },
        { GPS, sizeof GPS, FDT_MAVMSG_GPS_RAW_INT },
        { NED, sizeof NED, FDT_MAVMSG_LOCAL_POSITION_NED },
        { PRESS, sizeof PRESS, FDT_MAVMSG_SCALED_PRESSURE },
        { BAT, sizeof BAT, FDT_MAVMSG_BATTERY_STATUS },
    };
    for (size_t i = 0; i < sizeof F / sizeof F[0]; i++) {
        assert(feed(&p, F[i].b, F[i].n, &m) == 1);
        assert(m.msgid == F[i].id && m.version == 2);
        assert(m.sysid == 1 && m.compid == 1);
        assert(fdt_mav_dispatch(&ing, &m) == 1);
    }
    assert(fdt_mav_complete(&ing) == 1);

    const fdt_input_t *in = fdt_mav_input(&ing);
    assert(fabsf(in->ax_mps2 - 9.80665f) < 1e-3f);
    assert(fabsf(in->az_mps2 + 9.80665f) < 1e-3f);
    assert(fabsf(in->wx_rps - 0.5f) < 1e-6f);
    assert(fabsf(in->wy_rps + 0.25f) < 1e-6f);
    assert(fabsf(in->gps_lat_deg + 30.05f) < 1e-5f);
    assert(fabsf(in->gps_lon_deg + 51.17f) < 1e-5f);
    assert(fabsf(in->gps_alt_m - 3.0f) < 1e-6f);
    assert(fabsf(in->vn_mps - 1.25f) < 1e-6f);
    assert(fabsf(in->ve_mps + 0.5f) < 1e-6f);
    assert(fabsf(in->press_pa - 101325.0f) < 0.5f);
    assert(fabsf(in->temp_c - 22.5f) < 1e-4f);
    assert(fabsf(in->vbat_v - 18.5f) < 1e-4f);
    assert(fabsf(in->ibat_a - 6.2f) < 1e-4f);
    assert(p.frames == 5 && p.bad_crc == 0);

    printf("five pymavlink frames: I^t complete, %.2f V, %.1f Pa\n",
           (double)in->vbat_v, (double)in->press_pa);
}

static void test_v1_signed_and_corrupt(void)
{
    fdt_mav_parser_t p;
    memset(&p, 0, sizeof p);
    fdt_mav_msg_t m;

    assert(feed(&p, HB_V1, sizeof HB_V1, &m) == 1);
    assert(m.version == 1 && m.msgid == FDT_MAVMSG_HEARTBEAT);
    assert(m.payload[4] == 1 && m.payload[5] == 3); /* type, autopilot */

    /* A signed frame: the signature is framed and skipped, not checked. */
    assert(feed(&p, PRESS_SIGNED, sizeof PRESS_SIGNED, &m) == 1);
    assert(m.msgid == FDT_MAVMSG_SCALED_PRESSURE);

    /* One flipped payload byte: the frame is dropped, the next one parses. */
    uint8_t bad[sizeof IMU];
    memcpy(bad, IMU, sizeof IMU);
    bad[14] ^= 0x40u;
    assert(feed(&p, bad, sizeof bad, &m) == 0);
    assert(p.bad_crc == 1);
    assert(feed(&p, GPS, sizeof GPS, &m) == 1);
    assert(m.msgid == FDT_MAVMSG_GPS_RAW_INT);
}

/** A^t out, and back in through the same parser. */
static void test_encoders(void)
{
    uint8_t b[FDT_MAV_MAX_FRAME];
    const fdt_actuation_t a = { .throttle_pct = 50.0f, .cage_rad = 0.0f };
    const long n = fdt_mav_enc_rc_override(b, sizeof b, 7, 255, 190, 1, 1,
                                           &a);
    assert(n > 12);

    fdt_mav_parser_t p;
    memset(&p, 0, sizeof p);
    fdt_mav_msg_t m;
    assert(feed(&p, b, (size_t)n, &m) == 1);
    assert(m.msgid == FDT_MAVMSG_RC_CHANNELS_OVERRIDE);
    assert(m.sysid == 255 && m.compid == 190 && m.seq == 7);
    uint16_t thr = 0, cage = 0;
    fdt_mav_actuation_to_rc(&a, &thr, &cage);
    assert((m.payload[4] | (m.payload[5] << 8)) == thr);  /* channel 3 */
    assert((m.payload[0] | (m.payload[1] << 8)) == cage); /* channel 1 */
    assert((m.payload[2] | (m.payload[3] << 8)) == 0xFFFF);
    assert(m.payload[16] == 1 && m.payload[17] == 1);
    /* Channels 9 to 18 are zero, so MAVLink 2 truncates them. */
    assert(m.len == 18);

    const long h = fdt_mav_enc_heartbeat(b, sizeof b, 0, 255, 190);
    assert(h == 10 + 9 + 2);
    assert(feed(&p, b, (size_t)h, &m) == 1 && m.payload[4] == 6);

    assert(fdt_mav_enc_rc_override(b, 20, 0, 1, 1, 1, 1, &a) == -1);
    assert(fdt_mav_enc_rc_override(b, sizeof b, 0, 1, 1, 1, 1, NULL) == -1);

    printf("RC_CHANNELS_OVERRIDE for 50 %% throttle: %ld bytes, "
           "ch3 = %u us\n", n, (unsigned)thr);
}

/**
 * Figure 3 over UDP: a stand-in autopilot sends the telemetry frames to the
 * endpoint, which assembles I^t and answers with A^t to the sender.
 */
static void test_udp_link(void)
{
    const uint16_t port = 24550;
    fdt_mav_udp_t link;
    assert(fdt_mav_udp_open(&link, port) == 0);

    const fdt_actuation_t a = { .throttle_pct = 75.0f, .cage_rad = -0.3f };
    assert(fdt_mav_udp_send_act(&link, &a) == -1); /* nobody heard yet */

    const int ap = socket(AF_INET, SOCK_DGRAM, 0);
    assert(ap >= 0);
    struct sockaddr_in to;
    memset(&to, 0, sizeof to);
    to.sin_family = AF_INET;
    to.sin_port = htons(port);
    to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    const struct { const uint8_t *b; size_t n; } F[] = {
        { IMU, sizeof IMU }, { GPS, sizeof GPS }, { NED, sizeof NED },
        { PRESS, sizeof PRESS }, { BAT, sizeof BAT },
    };
    for (size_t i = 0; i < sizeof F / sizeof F[0]; i++) {
        assert(sendto(ap, F[i].b, F[i].n, 0, (struct sockaddr *)&to,
                      sizeof to) == (ssize_t)F[i].n);
    }

    fdt_mav_ingest_t ing;
    fdt_mav_init(&ing);
    int got = 0;
    for (int tries = 0; tries < 50 && got < 5; tries++) {
        const int r = fdt_mav_udp_poll(&link, &ing, 20);
        assert(r >= 0);
        got += r;
    }
    assert(got == 5 && fdt_mav_complete(&ing) == 1);
    assert(link.target_sys == 1 && link.target_comp == 1);

    /* A^t goes back to the address the telemetry came from. */
    assert(fdt_mav_udp_heartbeat(&link) == 0);
    assert(fdt_mav_udp_send_act(&link, &a) == 0);

    fdt_mav_parser_t p;
    memset(&p, 0, sizeof p);
    fdt_mav_msg_t m;
    int override = 0, heartbeat = 0;
    for (int k = 0; k < 2; k++) {
        uint8_t b[512];
        const ssize_t n = recv(ap, b, sizeof b, 0);
        assert(n > 0);
        for (ssize_t i = 0; i < n; i++) {
            if (fdt_mav_parse(&p, b[i], &m)) {
                override += (m.msgid == FDT_MAVMSG_RC_CHANNELS_OVERRIDE);
                heartbeat += (m.msgid == FDT_MAVMSG_HEARTBEAT);
            }
        }
    }
    assert(override == 1 && heartbeat == 1);
    uint16_t thr = 0, cage = 0;
    fdt_mav_actuation_to_rc(&a, &thr, &cage);
    assert((m.payload[4] | (m.payload[5] << 8)) == thr);

    close(ap);
    fdt_mav_udp_close(&link);
    printf("UDP: 5 telemetry frames in, I^t complete; A^t back as "
           "RC_CHANNELS_OVERRIDE, ch3 = %u us\n", (unsigned)thr);
}

int main(void)
{
    test_crc();
    test_telemetry_stream();
    test_v1_signed_and_corrupt();
    test_encoders();
    test_udp_link();

    printf("test_mavlink_wire: ok\n");
    return 0;
}
