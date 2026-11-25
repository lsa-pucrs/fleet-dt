#ifndef FLEET_DT_KALMAN_H
#define FLEET_DT_KALMAN_H

#include <fleet_dt/model.h>

/**
 * @file kalman.h
 * @brief The operator's estimate of B^t from I^t, Section V-A.
 *
 * Section V-A: "Telemetry was collected from a real boat and compared to the
 * pose and attitude estimation, using a Kalman filter to generate B^t_i. Data
 * from sensors (I^t_i) were used unfiltered to achieve the lowest possible
 * latency from sensing to the actuation path."
 *
 * This is the first of those two paths. Each quantity is a small linear
 * Kalman filter, stepped once per DT tick:
 *
 * - position north and east, and altitude: constant velocity, state
 *   [p, v], measured by the GPS fix and the GPS velocity (v_N, v_E, v_D);
 * - roll and pitch: the gyro rate predicts, the accelerometer's gravity
 *   direction corrects;
 * - yaw: the gyro rate predicts, the tilt-compensated magnetometer heading
 *   corrects, with the innovation wrapped to +-180 degrees.
 *
 * Surge and sway come from the filtered ground velocity rotated into the
 * body frame by the filtered yaw; heave is the filtered vertical velocity;
 * the body rates pass through. Position is filtered on a tangent plane
 * anchored at the first fix.
 */

/** Configuration of the filters; fdt_kf_defaults() fills a set for a small
 *  hull. */
typedef struct {
    double accel_sigma;    /**< Process: horizontal acceleration, m/s^2. */
    double gps_pos_sigma;  /**< Measurement: GPS position, m. */
    double gps_vel_sigma;  /**< Measurement: GPS velocity, m/s. */
    double gyro_sigma;     /**< Process: gyro rate noise, deg/s. */
    double tilt_sigma;     /**< Measurement: roll/pitch from gravity, deg. */
    double heading_sigma;  /**< Measurement: magnetometer heading, deg. */
    double declination_deg;/**< Magnetic declination of the site, deg,
                                east positive; yaw is true heading. */
} fdt_kf_config_t;

/** One constant-velocity axis: state [p, v] and its covariance. */
typedef struct {
    double x[2];
    double p[2][2];
} fdt_kf_cv_t;

/** One angle axis: state angle (deg) and its variance. */
typedef struct {
    double a;
    double p;
} fdt_kf_angle_t;

/** The operator's filter for one vessel. */
typedef struct {
    fdt_kf_config_t cfg;
    double         dt_s;
    int            anchored;
    double         ref_lat;
    double         ref_lon;
    fdt_kf_cv_t    north;
    fdt_kf_cv_t    east;
    fdt_kf_cv_t    up;
    fdt_kf_angle_t roll;
    fdt_kf_angle_t pitch;
    fdt_kf_angle_t yaw;
} fdt_kf_t;

/** @brief Noise figures for the Jundiá hulls' sensor set; declination 0. */
fdt_kf_config_t fdt_kf_defaults(void);

/**
 * @brief Arms the filter.
 * @param dt_s   Step period, seconds (the 125 ms tick).
 * @param cfg    Configuration; NULL takes fdt_kf_defaults().
 * @return 0, or -1 on a NULL filter or a non-positive period.
 */
int fdt_kf_init(fdt_kf_t *kf, double dt_s, const fdt_kf_config_t *cfg);

/**
 * @brief One predict-and-correct step on one sample of I^t.
 * @param in   The sensor sample.
 * @param out  The estimate of B^t.
 * @return 0, or -1 on a NULL argument.
 */
int fdt_kf_step(fdt_kf_t *kf, const fdt_input_t *in, fdt_state_t *out);

#endif /* FLEET_DT_KALMAN_H */
