/**
 * @file test_kalman.c
 * @brief The operator's Kalman estimate of B^t (Section V-A), against a
 *        known trajectory seen through noisy sensors.
 */
#include <fleet_dt/geo.h>
#include <fleet_dt/kalman.h>

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define DT      0.125
#define TICKS   400
#define DEG     (3.14159265358979 / 180.0)
#define G       9.80665

/* The truth: 1 m/s on a 45 degree heading, heeled 5 degrees, trimmed -3. */
#define LAT0    (-30.05)
#define LON0    (-51.17)
#define SPEED   1.0
#define PSI     45.0
#define PHI     5.0
#define THETA   (-3.0)

/* The geomagnetic field near Porto Alegre, NED, microtesla. */
static const double M_NED[3] = { 17.0, -3.0, -14.0 };

static uint32_t g_seed = 20261125u;

/** Uniform in [-1, 1], deterministic. */
static double noise(void)
{
    g_seed = g_seed * 1664525u + 1013904223u;
    return (double)(g_seed >> 8) / (double)(1u << 23) - 1.0;
}

/** v_body = R^T v_ned, R = Rz(psi) Ry(theta) Rx(phi). */
static void to_body(const double v[3], double out[3])
{
    const double cf = cos(PHI * DEG), sf = sin(PHI * DEG);
    const double ct = cos(THETA * DEG), st = sin(THETA * DEG);
    const double cp = cos(PSI * DEG), sp = sin(PSI * DEG);
    const double r[3][3] = {
        { ct * cp, sf * st * cp - cf * sp, cf * st * cp + sf * sp },
        { ct * sp, sf * st * sp + cf * cp, cf * st * sp - sf * cp },
        { -st,     sf * ct,                cf * ct },
    };
    for (int i = 0; i < 3; i++) {
        out[i] = r[0][i] * v[0] + r[1][i] * v[1] + r[2][i] * v[2];
    }
}

int main(void)
{
    fdt_kf_t kf;
    assert(fdt_kf_init(&kf, DT, NULL) == 0);
    assert(fdt_kf_init(NULL, DT, NULL) == -1);
    assert(fdt_kf_init(&kf, 0.0, NULL) == -1);
    fdt_kf_config_t cfg = fdt_kf_defaults();
    cfg.declination_deg = atan2(M_NED[1], M_NED[0]) / DEG;
    assert(fdt_kf_init(&kf, DT, &cfg) == 0);

    const double vn = SPEED * cos(PSI * DEG);
    const double ve = SPEED * sin(PSI * DEG);
    const double m_lon = fdt_geo_m_per_deg_lon(LAT0);

    double f_body[3];
    const double g_ned[3] = { 0.0, 0.0, -G }; /* specific force at rest */
    to_body(g_ned, f_body);
    double m_body[3];
    to_body(M_NED, m_body);

    double raw_sq = 0.0, est_sq = 0.0;
    int counted = 0;
    fdt_state_t b = {0};

    for (int k = 0; k < TICKS; k++) {
        const double t = k * DT;
        const double north = vn * t, east = ve * t;

        fdt_input_t in = {0};
        const double gn = north + 2.5 * noise(), ge = east + 2.5 * noise();
        in.gps_lat_deg = (float)(LAT0 + gn / FDT_M_PER_DEG_LAT);
        in.gps_lon_deg = (float)(LON0 + ge / m_lon);
        in.gps_alt_m   = (float)(3.0 + 1.0 * noise());
        in.vn_mps  = (float)(vn + 0.1 * noise());
        in.ve_mps  = (float)(ve + 0.1 * noise());
        in.vd_mps  = (float)(0.1 * noise());
        in.ax_mps2 = (float)(f_body[0] + 0.2 * noise());
        in.ay_mps2 = (float)(f_body[1] + 0.2 * noise());
        in.az_mps2 = (float)(f_body[2] + 0.2 * noise());
        in.wx_rps  = (float)(0.01 * noise());
        in.wy_rps  = (float)(0.01 * noise());
        in.wz_rps  = (float)(0.01 * noise());
        in.mx_ut   = (float)(m_body[0] + 0.5 * noise());
        in.my_ut   = (float)(m_body[1] + 0.5 * noise());
        in.mz_ut   = (float)(m_body[2] + 0.5 * noise());

        assert(fdt_kf_step(&kf, &in, &b) == 0);

        if (k >= TICKS / 2) {
            double e_raw, n_raw, e_est, n_est;
            fdt_geo_offset(LAT0, LON0, in.gps_lat_deg, in.gps_lon_deg,
                           &e_raw, &n_raw);
            fdt_geo_offset(LAT0, LON0, b.lat_deg, b.lon_deg, &e_est, &n_est);
            raw_sq += (e_raw - east) * (e_raw - east) +
                      (n_raw - north) * (n_raw - north);
            est_sq += (e_est - east) * (e_est - east) +
                      (n_est - north) * (n_est - north);
            counted++;
        }
    }

    const double raw_rms = sqrt(raw_sq / counted);
    const double est_rms = sqrt(est_sq / counted);
    printf("position error rms: raw GPS %.2f m, Kalman %.2f m\n",
           raw_rms, est_rms);
    printf("attitude: roll %.2f, pitch %.2f, yaw %.2f deg "
           "(truth %.1f, %.1f, %.1f)\n", (double)b.roll_deg,
           (double)b.pitch_deg, (double)b.yaw_deg, PHI, THETA, PSI);
    printf("body velocity: surge %.3f, sway %.3f m/s\n",
           (double)b.surge_mps, (double)b.sway_mps);

    assert(est_rms < 0.5 * raw_rms);
    assert(fabs(b.roll_deg - PHI) < 1.0);
    assert(fabs(b.pitch_deg - THETA) < 1.0);
    assert(fabs(b.yaw_deg - PSI) < 2.0);
    assert(fabs(b.surge_mps - SPEED) < 0.05);
    assert(fabs(b.sway_mps) < 0.05);
    assert(fabs(b.alt_m - 3.0) < 0.5);

    assert(fdt_kf_step(NULL, NULL, &b) == -1);

    printf("test_kalman: ok\n");
    return 0;
}
