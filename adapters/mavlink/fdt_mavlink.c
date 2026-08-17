#include "fdt_mavlink.h"

#include <stddef.h>

/** Standard gravity, for the milli-g to m/s^2 conversion. */
#define G_MPS2 9.80665f

void fdt_mav_init(fdt_mav_ingest_t *ing)
{
    if (ing == NULL) {
        return;
    }
    ing->input = (fdt_input_t){0};
    ing->have  = 0;
}

void fdt_mav_on_scaled_imu(fdt_mav_ingest_t *ing,
                           int16_t xacc, int16_t yacc, int16_t zacc,
                           int16_t xgyro, int16_t ygyro, int16_t zgyro,
                           int16_t xmag, int16_t ymag, int16_t zmag)
{
    if (ing == NULL) {
        return;
    }

    /* Milli-g to m/s^2. */
    ing->input.ax_mps2 = (float)xacc * G_MPS2 / 1000.0f;
    ing->input.ay_mps2 = (float)yacc * G_MPS2 / 1000.0f;
    ing->input.az_mps2 = (float)zacc * G_MPS2 / 1000.0f;

    /* Milliradians per second to rad/s. Table I gives omega in rad/s. */
    ing->input.wx_rps = (float)xgyro / 1000.0f;
    ing->input.wy_rps = (float)ygyro / 1000.0f;
    ing->input.wz_rps = (float)zgyro / 1000.0f;

    /* Milligauss to microtesla: 1 gauss is 100 uT, so 1 mgauss is 0.1 uT. */
    ing->input.mx_ut = (float)xmag * 0.1f;
    ing->input.my_ut = (float)ymag * 0.1f;
    ing->input.mz_ut = (float)zmag * 0.1f;

    ing->have |= FDT_MAV_HAVE_IMU;
}

void fdt_mav_on_gps_raw(fdt_mav_ingest_t *ing, int32_t lat, int32_t lon,
                        int32_t alt_mm)
{
    if (ing == NULL) {
        return;
    }

    /* MAVLink carries degrees scaled by 1e7 to keep them integral. */
    ing->input.gps_lat_deg = (float)((double)lat * 1e-7);
    ing->input.gps_lon_deg = (float)((double)lon * 1e-7);
    ing->input.gps_alt_m   = (float)alt_mm / 1000.0f;

    ing->have |= FDT_MAV_HAVE_GPS;
}

void fdt_mav_on_local_ned(fdt_mav_ingest_t *ing, float vx, float vy, float vz)
{
    if (ing == NULL) {
        return;
    }

    /* Already m/s, and already north-east-down, which is what Table I wants. */
    ing->input.vn_mps = vx;
    ing->input.ve_mps = vy;
    ing->input.vd_mps = vz;

    ing->have |= FDT_MAV_HAVE_NED;
}

void fdt_mav_on_scaled_pressure(fdt_mav_ingest_t *ing, float press_abs_hpa,
                                int16_t temperature_c)
{
    if (ing == NULL) {
        return;
    }

    ing->input.press_pa = press_abs_hpa * 100.0f;    /* hPa to Pa */
    ing->input.temp_c   = (float)temperature_c / 100.0f; /* c-deg to deg */

    ing->have |= FDT_MAV_HAVE_PRESSURE;
}

void fdt_mav_on_battery(fdt_mav_ingest_t *ing, uint16_t voltage_mv,
                        int16_t current_battery)
{
    if (ing == NULL) {
        return;
    }

    ing->input.vbat_v = (float)voltage_mv / 1000.0f;

    ing->input.ibat_a = (current_battery < 0)
                      ? 0.0f
                      : (float)current_battery / 100.0f;

    ing->have |= FDT_MAV_HAVE_BATTERY;
}

void fdt_mav_attach_views(fdt_mav_ingest_t *ing, const void *left,
                          const void *right)
{
    if (ing == NULL) {
        return;
    }
    ing->input.x_left  = left;
    ing->input.x_right = right;
}

int fdt_mav_complete(const fdt_mav_ingest_t *ing)
{
    if (ing == NULL) {
        return 0;
    }
    return (ing->have & FDT_MAV_HAVE_ALL) == FDT_MAV_HAVE_ALL;
}

const fdt_input_t *fdt_mav_input(const fdt_mav_ingest_t *ing)
{
    return (ing == NULL) ? NULL : &ing->input;
}

/** Maps a value in [-1, 1] onto an RC pulse width, clamping at the ends. */
static uint16_t to_pulse(float unit)
{
    if (unit > 1.0f) {
        unit = 1.0f;
    }
    if (unit < -1.0f) {
        unit = -1.0f;
    }
    return (uint16_t)((float)FDT_RC_MID + unit * (float)FDT_RC_SPAN);
}

void fdt_mav_actuation_to_rc(const fdt_actuation_t *a, uint16_t *chan_throttle,
                             uint16_t *chan_cage)
{
    if (a == NULL) {
        return;
    }

    if (chan_throttle != NULL) {
        float unit = a->throttle_pct / 100.0f;
        if (unit < 0.0f) { unit = 0.0f; }
        if (unit > 1.0f) { unit = 1.0f; }
        *chan_throttle = (uint16_t)(1000.0f + unit * 1000.0f);
    }

    if (chan_cage != NULL) {
        *chan_cage = to_pulse(a->cage_rad / FDT_CAGE_LIMIT_RAD);
    }
}
