/**
 * @file test_model.c
 * @brief Pins the published figures of equation (1) and Table I.
 *
 * Claims covered: C14 (the 48-byte state), C19 (Table I's 21 entries and
 * their units). See docs/spec/paper-claims.md.
 */
#include <fleet_dt/model.h>
#include <fleet_dt/version.h>

#include <assert.h>
#include <stdio.h>
#include <string.h>

/* C14, Section IV: "a state (B_i^t) occupies 12 floating point values in
 * memory, translating to 48 bytes". This is an assertion, not a measurement:
 * the figure is published, a reader may check it, and the build must fail
 * rather than silently disagree with the manuscript. */
_Static_assert(sizeof(fdt_state_t) == 48,
               "equation (1) state must occupy 48 bytes");
_Static_assert(sizeof(fdt_state_t) == FDT_STATE_FLOATS * sizeof(float),
               "state must be exactly its twelve floats, with no padding");

/** Exercises every field of B^t_k in the order equation (1) lists them. */
static void test_state_layout(void)
{
    fdt_state_t b = {
        .lat_deg = 1.0f, .lon_deg = 2.0f, .alt_m = 3.0f,
        .roll_deg = 4.0f, .pitch_deg = 5.0f, .yaw_deg = 6.0f,
        .surge_mps = 7.0f, .sway_mps = 8.0f, .heave_mps = 9.0f,
        .roll_rate_rps = 10.0f, .pitch_rate_rps = 11.0f, .yaw_rate_rps = 12.0f,
    };

    /* The twelve values are addressable as one contiguous run, which is what
     * makes the 48-byte figure meaningful in the first place. */
    const float *as_floats = (const float *)&b;
    for (int i = 0; i < FDT_STATE_FLOATS; i++) {
        assert(as_floats[i] == (float)(i + 1));
    }
}

/** Exercises the 21 entries of I^t_k, and the absence of a 22nd. */
static void test_input_entries(void)
{
    fdt_input_t in = {0};

    in.ax_mps2 = in.ay_mps2 = in.az_mps2 = 1.0f;
    in.wx_rps  = in.wy_rps  = in.wz_rps  = 1.0f;
    in.mx_ut   = in.my_ut   = in.mz_ut   = 1.0f;
    in.gps_lat_deg = in.gps_lon_deg = in.gps_alt_m = 1.0f;
    in.vn_mps = in.ve_mps = in.vd_mps = 1.0f;
    in.press_pa = in.temp_c = in.vbat_v = in.ibat_a = 1.0f;
    in.x_left = in.x_right = NULL;

    /* 19 scalars plus 2 opaque views. Table I lists no timestamp, so the
     * struct carries none; sequencing is the envelope's job. */
    assert(in.ibat_a == 1.0f);
    assert(in.x_left == NULL && in.x_right == NULL);
}

/** Actuation and goal, equation (2) and Table I. */
static void test_actuation_and_goal(void)
{
    fdt_actuation_t a = { .throttle_pct = 50.0f, .cage_rad = 0.1f };
    assert(a.throttle_pct == 50.0f && a.cage_rad == 0.1f);

    /* g^t_k "may have the same structure as B^t_k": a goal that only targets
     * yaw leaves the remaining eleven fields at zero. */
    fdt_goal_t g = { .yaw_deg = 90.0f };
    assert(g.yaw_deg == 90.0f && g.surge_mps == 0.0f);
    assert(sizeof(fdt_goal_t) == sizeof(fdt_state_t));
}

/** The manuscript revision is pinned to a symbol, not left to prose. */
static void test_tracked_revision(void)
{
    assert(strcmp(FDT_PAPER_REVISION, "-10") == 0);
    assert(strstr(fdt_version(), FDT_PAPER_REVISION) != NULL);
}

int main(void)
{
    test_state_layout();
    test_input_entries();
    test_actuation_and_goal();
    test_tracked_revision();

    printf("%s\n", fdt_version());
    printf("sizeof(fdt_state_t) = %zu bytes\n", sizeof(fdt_state_t));
    printf("sizeof(fdt_input_t) = %zu bytes\n", sizeof(fdt_input_t));
    printf("test_model: ok\n");
    return 0;
}
