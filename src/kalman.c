#include <fleet_dt/kalman.h>

#include <fleet_dt/geo.h>

#include <math.h>
#include <stddef.h>

#define RAD2DEG 57.29577951308232

/** Wraps degrees to (-180, 180]. */
static double wrap180(double a)
{
    a = fmod(a + 180.0, 360.0);
    if (a <= 0.0) {
        a += 360.0;
    }
    return a - 180.0;
}

fdt_kf_config_t fdt_kf_defaults(void)
{
    return (fdt_kf_config_t){ .accel_sigma   = 0.5,
                             .gps_pos_sigma = 2.5,
                             .gps_vel_sigma = 0.1,
                             .gyro_sigma    = 1.0,
                             .tilt_sigma    = 3.0,
                             .heading_sigma = 5.0,
                             .declination_deg = 0.0 };
}

static void cv_reset(fdt_kf_cv_t *f, double p0, double v0, double var)
{
    f->x[0] = p0;
    f->x[1] = v0;
    f->p[0][0] = var;
    f->p[0][1] = 0.0;
    f->p[1][0] = 0.0;
    f->p[1][1] = var;
}

/** Constant-velocity predict with white acceleration noise. */
static void cv_predict(fdt_kf_cv_t *f, double dt, double q_acc)
{
    const double q = q_acc * q_acc;
    f->x[0] += dt * f->x[1];

    const double p00 = f->p[0][0], p01 = f->p[0][1];
    const double p10 = f->p[1][0], p11 = f->p[1][1];
    f->p[0][0] = p00 + dt * (p10 + p01) + dt * dt * p11 +
                 q * dt * dt * dt * dt / 4.0;
    f->p[0][1] = p01 + dt * p11 + q * dt * dt * dt / 2.0;
    f->p[1][0] = p10 + dt * p11 + q * dt * dt * dt / 2.0;
    f->p[1][1] = p11 + q * dt * dt;
}

/** Scalar update of component @p i (0 position, 1 velocity). */
static void cv_update(fdt_kf_cv_t *f, int i, double z, double r)
{
    const double s = f->p[i][i] + r;
    const double k0 = f->p[0][i] / s;
    const double k1 = f->p[1][i] / s;
    const double y = z - f->x[i];

    f->x[0] += k0 * y;
    f->x[1] += k1 * y;

    const double pi0 = f->p[i][0], pi1 = f->p[i][1];
    f->p[0][0] -= k0 * pi0;
    f->p[0][1] -= k0 * pi1;
    f->p[1][0] -= k1 * pi0;
    f->p[1][1] -= k1 * pi1;
}

static void angle_step(fdt_kf_angle_t *f, double rate_dps, double dt,
                       double q_dps, double z_deg, double r_deg)
{
    f->a = wrap180(f->a + rate_dps * dt);
    f->p += (q_dps * dt) * (q_dps * dt);

    const double k = f->p / (f->p + r_deg * r_deg);
    f->a = wrap180(f->a + k * wrap180(z_deg - f->a));
    f->p *= (1.0 - k);
}

int fdt_kf_init(fdt_kf_t *kf, double dt_s, const fdt_kf_config_t *cfg)
{
    if (kf == NULL || !(dt_s > 0.0)) {
        return -1;
    }
    kf->cfg      = (cfg != NULL) ? *cfg : fdt_kf_defaults();
    kf->dt_s     = dt_s;
    kf->anchored = 0;
    kf->ref_lat  = 0.0;
    kf->ref_lon  = 0.0;
    cv_reset(&kf->north, 0.0, 0.0, 1e6);
    cv_reset(&kf->east, 0.0, 0.0, 1e6);
    cv_reset(&kf->up, 0.0, 0.0, 1e6);
    kf->roll  = (fdt_kf_angle_t){ 0.0, 1e4 };
    kf->pitch = (fdt_kf_angle_t){ 0.0, 1e4 };
    kf->yaw   = (fdt_kf_angle_t){ 0.0, 1e4 };
    return 0;
}

int fdt_kf_step(fdt_kf_t *kf, const fdt_input_t *in, fdt_state_t *out)
{
    if (kf == NULL || in == NULL || out == NULL) {
        return -1;
    }
    const fdt_kf_config_t *n = &kf->cfg;
    const double dt = kf->dt_s;

    /* Position and velocity on the tangent plane of the first fix. */
    if (!kf->anchored) {
        kf->ref_lat  = (double)in->gps_lat_deg;
        kf->ref_lon  = (double)in->gps_lon_deg;
        kf->anchored = 1;
    }
    double east_m = 0.0;
    double north_m = 0.0;
    fdt_geo_offset(kf->ref_lat, kf->ref_lon, (double)in->gps_lat_deg,
                   (double)in->gps_lon_deg, &east_m, &north_m);

    const double rp = n->gps_pos_sigma * n->gps_pos_sigma;
    const double rv = n->gps_vel_sigma * n->gps_vel_sigma;

    cv_predict(&kf->north, dt, n->accel_sigma);
    cv_update(&kf->north, 0, north_m, rp);
    cv_update(&kf->north, 1, (double)in->vn_mps, rv);

    cv_predict(&kf->east, dt, n->accel_sigma);
    cv_update(&kf->east, 0, east_m, rp);
    cv_update(&kf->east, 1, (double)in->ve_mps, rv);

    cv_predict(&kf->up, dt, n->accel_sigma);
    cv_update(&kf->up, 0, (double)in->gps_alt_m, rp);
    cv_update(&kf->up, 1, -(double)in->vd_mps, rv);

    /* Attitude: gyro predicts, gravity and the magnetic field correct.
     * The accelerometer reads specific force in the body frame (NED), as
     * ArduPilot's SCALED_IMU does: a level hull at rest reads az = -g. */
    const double ax = in->ax_mps2, ay = in->ay_mps2, az = in->az_mps2;
    const double roll_z  = atan2(-ay, -az) * RAD2DEG;
    const double pitch_z = atan2(ax, sqrt(ay * ay + az * az)) * RAD2DEG;

    angle_step(&kf->roll, in->wx_rps * RAD2DEG, dt, n->gyro_sigma,
               roll_z, n->tilt_sigma);
    angle_step(&kf->pitch, in->wy_rps * RAD2DEG, dt, n->gyro_sigma,
               pitch_z, n->tilt_sigma);

    /* Tilt-compensated heading, clockwise from magnetic north. */
    const double phi = kf->roll.a / RAD2DEG;
    const double th  = kf->pitch.a / RAD2DEG;
    const double mx = in->mx_ut, my = in->my_ut, mz = in->mz_ut;
    const double xh = mx * cos(th) + my * sin(phi) * sin(th) +
                      mz * cos(phi) * sin(th);
    const double yh = my * cos(phi) - mz * sin(phi);
    const double heading_z = wrap180(atan2(-yh, xh) * RAD2DEG +
                                     n->declination_deg);

    if (mx != 0.0 || my != 0.0 || mz != 0.0) {
        angle_step(&kf->yaw, in->wz_rps * RAD2DEG, dt, n->gyro_sigma,
                   heading_z, n->heading_sigma);
    } else {
        /* No magnetometer: the gyro alone carries yaw. */
        kf->yaw.a = wrap180(kf->yaw.a + in->wz_rps * RAD2DEG * dt);
        kf->yaw.p += (n->gyro_sigma * dt) * (n->gyro_sigma * dt);
    }

    /* Ground velocity into the body frame by the filtered yaw. */
    const double psi = kf->yaw.a / RAD2DEG;
    const double vn = kf->north.x[1];
    const double ve = kf->east.x[1];

    fdt_state_t b = {0};
    b.lat_deg = (float)(kf->ref_lat + kf->north.x[0] / FDT_M_PER_DEG_LAT);
    const double m_lon = fdt_geo_m_per_deg_lon(kf->ref_lat);
    b.lon_deg = (float)(kf->ref_lon +
                        ((m_lon > 0.0) ? kf->east.x[0] / m_lon : 0.0));
    b.alt_m          = (float)kf->up.x[0];
    b.roll_deg       = (float)kf->roll.a;
    b.pitch_deg      = (float)kf->pitch.a;
    b.yaw_deg        = (float)kf->yaw.a;
    b.surge_mps      = (float)(vn * cos(psi) + ve * sin(psi));
    b.sway_mps       = (float)(-vn * sin(psi) + ve * cos(psi));
    b.heave_mps      = (float)(-kf->up.x[1]);
    b.roll_rate_rps  = in->wx_rps;
    b.pitch_rate_rps = in->wy_rps;
    b.yaw_rate_rps   = in->wz_rps;

    *out = b;
    return 0;
}
