#ifndef FLEET_DT_MODEL_H
#define FLEET_DT_MODEL_H

/**
 * @file model.h
 * @brief The four model types of Section IV, equation (1) and Table I.
 *
 * From "A Digital Twin Model and Architecture for Monitoring and Controlling
 * Fleets of Autonomous Unmanned Surface Vehicles", ICECS 2026, manuscript
 * revision -10.
 *
 * Units are copied from Table I rather than normalised. The paper mixes
 * conventions — attitude angles in degrees, attitude rates in rad/s — and the
 * mix is preserved here because normalising it would make the code disagree
 * with the published table. Every field carries its unit in its name so the
 * mix cannot be applied by accident: any transition that integrates a rate
 * into an angle has to cross the units rather than add them raw.
 *
 * @note fdt_state_t derives from dt-daemon/include/boat.h by Anderson
 *       Domingues in lsa-pucrs/boat-digital-twin. That repository is private,
 *       so this is a credit, not a link a reader can follow.
 */

/**
 * The twelve floating-point values of equation (1).
 *
 * Section IV publishes the width they occupy — "a state (B_i^t) occupies 12
 * floating point values in memory, translating to 48 bytes" — and
 * tests/test_model.c pins sizeof(fdt_state_t) to it with a static assertion.
 */
#define FDT_STATE_FLOATS 12

/**
 * @brief I^t_k of equation (1): the environment input of vessel k at time t.
 *
 * Exactly the 21 entries Table I lists and nothing else. In particular there
 * is no timestamp field, because Table I declares none; sequencing lives in
 * the wire envelope, not in the model.
 *
 * The two stereo camera views are carried as opaque pointers because they are
 * images, not scalars. This library never reads through them: Section III
 * moves the camera feed off MQTT and onto RTSP, so the views are re-attached
 * on the high-speed path rather than travelling with the telemetry.
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
 *
 * Twelve variables in the order the paper's matrix lists them, all single
 * precision. The precision is not a taste call: Section IV fixes the state at
 * 48 bytes, and twelve four-byte floats are exactly that. Widening any field
 * would make the published figure false.
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
 *
 * The Jundia boat steers by rotating its propulsion cage; it has no rudder,
 * which is why there is no rudder field to leave at zero.
 */
typedef struct {
    float throttle_pct; /**< tau — throttle, percent. */
    float cage_rad;     /**< alpha — propulsion cage angle, radians. */
} fdt_actuation_t;

/**
 * @brief g^t_k of Section IV: the mission goal.
 *
 * The paper states it "may have the same structure as B^t_k, similar to a
 * setpoint in control systems", so it is an alias rather than a distinct
 * type. A goal that only constrains yaw simply leaves the other eleven
 * fields untouched.
 */
typedef fdt_state_t fdt_goal_t;

#endif /* FLEET_DT_MODEL_H */
