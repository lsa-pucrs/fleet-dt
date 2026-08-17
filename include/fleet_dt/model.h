#ifndef FLEET_DT_MODEL_H
#define FLEET_DT_MODEL_H

/**
 * @file model.h
 * @brief The four model types of Section IV, equation (1) and Table I.
 *
 * @note fdt_state_t derives from dt-daemon/include/boat.h by Anderson
 *       Domingues in lsa-pucrs/boat-digital-twin. That repository is private,
 *       so this is a credit, not a link a reader can follow.
 */

/**
 * The twelve floating-point values of equation (1).
 */
#define FDT_STATE_FLOATS 12

/**
 * @brief I^t_k of equation (1): the environment input of vessel k at time t.
 */
typedef struct {
    float ax_mps2;      /**< a_x — linear acceleration, X axis, m/s^2. */
    float ay_mps2;      /**< a_y — linear acceleration, Y axis, m/s^2. */
    float az_mps2;      /**< a_z — linear acceleration, Z axis, m/s^2. */

    float wx_rps;       /**< omega_x — angular velocity, X axis, rad/s. */
    float wy_rps;       /**< omega_y — angular velocity, Y axis, rad/s. */
    float wz_rps;       /**< omega_z — angular velocity, Z axis, rad/s. */

    float mx_ut;        /**< m_x — magnetic field, X axis, microtesla. */
    float my_ut;        /**< m_y — magnetic field, Y axis, microtesla. */
    float mz_ut;        /**< m_z — magnetic field, Z axis, microtesla. */

    float gps_lat_deg;  /**< phi_gps — GPS latitude, degrees. */
    float gps_lon_deg;  /**< lambda_gps — GPS longitude, degrees. */
    float gps_alt_m;    /**< h_gps — GPS altitude above mean sea level, m. */

    float vn_mps;       /**< v_N — velocity North, m/s. */
    float ve_mps;       /**< v_E — velocity East, m/s. */
    float vd_mps;       /**< v_D — velocity Down, m/s. */

    float press_pa;     /**< P — atmospheric pressure, pascal. */
    float temp_c;       /**< T — temperature, degrees Celsius. */
    float vbat_v;       /**< V_b — battery charge, volts. */
    float ibat_a;       /**< I_b — battery current consumption, amperes. */

    const void *x_left;  /**< X_left — left stereo view, opaque, may be NULL. */
    const void *x_right; /**< X_right — right stereo view, opaque, may be NULL. */
} fdt_input_t;

/**
 * @brief B^t_k of equation (1): the digital twin state.
 */
typedef struct {
    float lat_deg;        /**< phi — vessel latitude, degrees. */
    float lon_deg;        /**< lambda — vessel longitude, degrees. */
    float alt_m;          /**< h — altitude above mean sea level, metres. */

    float roll_deg;       /**< varphi — roll angle, degrees. */
    float pitch_deg;      /**< theta — pitch angle, degrees. */
    float yaw_deg;        /**< psi — yaw angle, degrees. */

    float surge_mps;      /**< u — surge velocity, m/s. */
    float sway_mps;       /**< v — sway velocity, m/s. */
    float heave_mps;      /**< w — heave velocity, m/s. */

    float roll_rate_rps;  /**< p — roll rate, rad/s. */
    float pitch_rate_rps; /**< q — pitch rate, rad/s. */
    float yaw_rate_rps;   /**< r — yaw rate, rad/s. */
} fdt_state_t;

/**
 * @brief A^t_k of equation (2) and Table I: the actuation pi produces.
 */
typedef struct {
    float throttle_pct; /**< tau — throttle, percent. */
    float cage_rad;     /**< alpha — propulsion cage angle, radians. */
} fdt_actuation_t;

/**
 * @brief g^t_k of Section IV: the mission goal.
 */
typedef fdt_state_t fdt_goal_t;

#endif /* FLEET_DT_MODEL_H */
