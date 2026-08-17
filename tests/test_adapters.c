/**
 * @file test_adapters.c
 * @brief The Ardupilot ingest mapping and the HSDT camera boundary.
 *
 * Claims covered: the Ardupilot and camera halves of C3, and C11 (the camera
 * feed kept off the MQTT path). See docs/spec/paper-claims.md.
 */
#include "mavlink/fdt_mavlink.h"
#include "rtsp/fdt_rtsp.h"

#include <fleet_dt/codec.h>

#include <assert.h>
#include <math.h>
#include <stdio.h>

/** Every row of the mapping table, checked with a known value. */
static void test_mavlink_conversions(void)
{
    fdt_mav_ingest_t ing;
    fdt_mav_init(&ing);

    assert(fdt_mav_complete(&ing) == 0);
    assert(fdt_mav_input(NULL) == NULL);
    assert(fdt_mav_complete(NULL) == 0);

    /* SCALED_IMU2. 1000 milli-g is exactly one standard gravity; 500 mrad/s
     * is 0.5 rad/s; 230 milligauss is 23 uT, a plausible southern field. */
    fdt_mav_on_scaled_imu(&ing, 1000, 0, -1000, 500, -250, 0, 230, 0, -140);
    const fdt_input_t *in = fdt_mav_input(&ing);

    assert(fabsf(in->ax_mps2 - 9.80665f) < 1e-4f);
    assert(fabsf(in->az_mps2 + 9.80665f) < 1e-4f);
    assert(fabsf(in->wx_rps - 0.5f) < 1e-6f);
    assert(fabsf(in->wy_rps + 0.25f) < 1e-6f);
    assert(fabsf(in->mx_ut - 23.0f) < 1e-4f);
    assert(fabsf(in->mz_ut + 14.0f) < 1e-4f);

    /* GPS_RAW_INT: degrees times 1e7, altitude in millimetres. */
    fdt_mav_on_gps_raw(&ing, -300500000, -511700000, 3000);
    assert(fabsf(in->gps_lat_deg + 30.05f) < 1e-4f);
    assert(fabsf(in->gps_lon_deg + 51.17f) < 1e-4f);
    assert(fabsf(in->gps_alt_m - 3.0f) < 1e-6f);

    /* LOCAL_POSITION_NED: already m/s, already NED. */
    fdt_mav_on_local_ned(&ing, 1.25f, -0.5f, 0.02f);
    assert(in->vn_mps == 1.25f && in->ve_mps == -0.5f && in->vd_mps == 0.02f);

    /* SCALED_PRESSURE: hPa to Pa, hundredths of a degree to degrees. */
    fdt_mav_on_scaled_pressure(&ing, 1013.25f, 2250);
    assert(fabsf(in->press_pa - 101325.0f) < 1e-1f);
    assert(fabsf(in->temp_c - 22.5f) < 1e-4f);

    /* BATTERY_STATUS: the 18.5 V pack of Section II, current in centiamps. */
    fdt_mav_on_battery(&ing, 18500, 620);
    assert(fabsf(in->vbat_v - 18.5f) < 1e-4f);
    assert(fabsf(in->ibat_a - 6.2f) < 1e-4f);

    assert(fdt_mav_complete(&ing) == 1);
}

/** A frame is incomplete until every family has arrived. */
static void test_mavlink_completeness(void)
{
    fdt_mav_ingest_t ing;
    fdt_mav_init(&ing);

    fdt_mav_on_scaled_imu(&ing, 0, 0, -1000, 0, 0, 0, 230, 0, 0);
    assert(fdt_mav_complete(&ing) == 0);
    fdt_mav_on_gps_raw(&ing, -300500000, -511700000, 3000);
    assert(fdt_mav_complete(&ing) == 0);
    fdt_mav_on_local_ned(&ing, 0.0f, 0.0f, 0.0f);
    assert(fdt_mav_complete(&ing) == 0);
    fdt_mav_on_scaled_pressure(&ing, 1013.25f, 2200);
    assert(fdt_mav_complete(&ing) == 0);

    /* Only the fifth family completes it. A partial frame would hand delta^e
     * zeros indistinguishable from real readings. */
    fdt_mav_on_battery(&ing, 18500, 600);
    assert(fdt_mav_complete(&ing) == 1);

    /* Attaching views does not affect completeness: they arrive over RTSP,
     * and their absence must not hold up a frame MAVLink has finished. */
    fdt_mav_init(&ing);
    fdt_mav_attach_views(&ing, (const void *)"L", (const void *)"R");
    assert(fdt_mav_complete(&ing) == 0);

    /* A reset clears everything. */
    fdt_mav_on_battery(&ing, 18500, 600);
    fdt_mav_init(&ing);
    assert(fdt_mav_complete(&ing) == 0);
    assert(fdt_mav_input(&ing)->vbat_v == 0.0f);

    /* Unknown current is reported as no draw, not as a negative one that
     * would read as charging. */
    fdt_mav_on_battery(&ing, 18500, -1);
    assert(fdt_mav_input(&ing)->ibat_a == 0.0f);
}

/** A^t back out as RC channel overrides. */
static void test_actuation_to_rc(void)
{
    uint16_t thr = 0;
    uint16_t cage = 0;

    /* Throttle spans the full range: 0 % is the low end, not neutral. */
    const fdt_actuation_t zero = { .throttle_pct = 0.0f, .cage_rad = 0.0f };
    fdt_mav_actuation_to_rc(&zero, &thr, &cage);
    assert(thr == 1000);
    assert(cage == FDT_RC_MID);

    const fdt_actuation_t half = { .throttle_pct = 50.0f, .cage_rad = 0.0f };
    fdt_mav_actuation_to_rc(&half, &thr, &cage);
    assert(thr == 1500);

    const fdt_actuation_t full = { .throttle_pct = 100.0f, .cage_rad = 0.0f };
    fdt_mav_actuation_to_rc(&full, &thr, &cage);
    assert(thr == 2000);

    /* The cage is centred, and clamps at the mechanical limit rather than
     * wrapping past it. */
    const fdt_actuation_t hard = { .throttle_pct = 0.0f,
                                   .cage_rad = FDT_CAGE_LIMIT_RAD };
    fdt_mav_actuation_to_rc(&hard, &thr, &cage);
    assert(cage == FDT_RC_MID + FDT_RC_SPAN);

    const fdt_actuation_t over = { .throttle_pct = 0.0f, .cage_rad = 99.0f };
    fdt_mav_actuation_to_rc(&over, &thr, &cage);
    assert(cage == FDT_RC_MID + FDT_RC_SPAN);

    const fdt_actuation_t under = { .throttle_pct = -50.0f,
                                    .cage_rad = -99.0f };
    fdt_mav_actuation_to_rc(&under, &thr, &cage);
    assert(thr == 1000);
    assert(cage == FDT_RC_MID - FDT_RC_SPAN);

    /* Either channel may be ignored. */
    fdt_mav_actuation_to_rc(&half, NULL, &cage);
    fdt_mav_actuation_to_rc(&half, &thr, NULL);
    fdt_mav_actuation_to_rc(NULL, &thr, &cage);
}

/** C11: the camera path is separate, and the codec never carries an image. */
static void test_rtsp_boundary(void)
{
    fdt_rtsp_fake_t fake;
    fdt_rtsp_t src = fdt_rtsp_fake(&fake, 1920, 1080);
    assert(src.self != NULL && src.next_frame != NULL);

    fdt_view_t left;
    fdt_view_t right;
    assert(src.next_frame(src.self, &left, &right) == 0);
    assert(left.data != NULL && right.data != NULL);
    assert(left.width == 1920 && left.height == 1080);
    assert(left.seq == 1 && right.seq == 1);

    /* Successive frames differ, and the two views differ from each other. */
    const uint8_t first = left.data[0];
    assert(src.next_frame(src.self, &left, &right) == 0);
    assert(left.seq == 2);
    assert(left.data[0] != first || left.data[1] != right.data[1]);

    /* An input that crossed the wire carries sentinels, not images. */
    fdt_input_t on_boat = {0};
    on_boat.x_left  = left.data;
    on_boat.x_right = right.data;

    uint8_t buf[FDT_WIRE_INPUT_BYTES];
    assert(fdt_enc_input(&on_boat, buf, sizeof buf) == FDT_WIRE_INPUT_BYTES);

    fdt_input_t at_ground = {0};
    assert(fdt_dec_input(buf, sizeof buf, &at_ground) ==
           FDT_WIRE_INPUT_BYTES);
    assert(at_ground.x_left == FDT_VIEW_PRESENT);
    assert(at_ground.x_right == FDT_VIEW_PRESENT);
    assert(fdt_rtsp_pending(&at_ground) == 1);

    /* The high-speed path resolves the sentinels into real frames. */
    assert(fdt_rtsp_attach(&at_ground, &left, &right) == 0);
    assert(at_ground.x_left == (const void *)left.data);
    assert(fdt_rtsp_pending(&at_ground) == 0);

    /* No frame available clears the field rather than leaving a sentinel a
     * reader might dereference. */
    const fdt_view_t empty = {0};
    assert(fdt_rtsp_attach(&at_ground, &empty, NULL) == 0);
    assert(at_ground.x_left == NULL && at_ground.x_right == NULL);
    assert(fdt_rtsp_pending(&at_ground) == 0);

    assert(fdt_rtsp_attach(NULL, &left, &right) == -1);
    assert(fdt_rtsp_pending(NULL) == 0);

    src.close(src.self);

    const fdt_rtsp_t none = fdt_rtsp_fake(NULL, 0, 0);
    assert(none.self == NULL);
}

int main(void)
{
    test_mavlink_conversions();
    test_mavlink_completeness();
    test_actuation_to_rc();
    test_rtsp_boundary();

    printf("test_adapters: ok\n");
    return 0;
}
