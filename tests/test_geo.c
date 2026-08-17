/**
 * @file test_geo.c
 * @brief The geodetic-to-local mapping, and the bug it was extracted from.
 *
 * Not a paper claim. This is a regression guard: the WeBots controller used to
 * do this arithmetic inline and did it wrong, scaling an absolute longitude
 * into a world coordinate. Every vessel then rendered thousands of kilometres
 * outside a world a thousand metres across, and nothing failed — the frames
 * kept stepping, the counters stayed green, and the only symptom was an empty
 * lagoon in a 3D view no test could look at.
 *
 * Moving the arithmetic into the library is what makes these checks possible.
 */
#include <fleet_dt/geo.h>

#include <assert.h>
#include <math.h>
#include <stdio.h>

/* -std=c18 without _GNU_SOURCE does not expose M_PI; it is not in ISO C. */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/** The Jundiá sampling area of Section II: 30 degrees south, 51 west. */
#define REF_LAT (-30.05)
#define REF_LON (-51.17)

/** A point offset from its own reference is at the origin, not in orbit. */
static void test_reference_is_the_origin(void)
{
    double east = 1.0;
    double north = 1.0;

    fdt_geo_offset(REF_LAT, REF_LON, REF_LAT, REF_LON, &east, &north);

    assert(fabs(east) < 1e-9);
    assert(fabs(north) < 1e-9);

    /* This is the whole of the old bug. Multiplying the absolute coordinate
     * instead of the difference put the vessel here: */
    const double wrong_x = REF_LON * FDT_M_PER_DEG_LAT;
    const double wrong_z = REF_LAT * FDT_M_PER_DEG_LAT;
    assert(fabs(wrong_x) > 5.0e6);   /* 5696 km west  */
    assert(fabs(wrong_z) > 3.0e6);   /* 3345 km south */

    printf("absolute scaling would place the origin at %.0f, %.0f km\n",
           wrong_x / 1000.0, wrong_z / 1000.0);
}

/** Both axes scale the way the ellipsoid does. */
static void test_scales(void)
{
    /* A tenth of a degree north is 11.132 km, everywhere. */
    double north = 0.0;
    fdt_geo_offset(REF_LAT, REF_LON, REF_LAT + 0.1, REF_LON, NULL, &north);
    assert(fabs(north - 11132.0) < 1.0);

    /* A tenth of a degree east is shorter, by the cosine of the latitude. At
     * 30 degrees south that is 0.866, so about 9.64 km. */
    double east = 0.0;
    fdt_geo_offset(REF_LAT, REF_LON, REF_LAT, REF_LON + 0.1, &east, NULL);
    assert(east > 9500.0 && east < 9700.0);
    assert(east < north);

    /* The cosine factor at the equator is one, and it vanishes at the pole. */
    assert(fabs(fdt_geo_m_per_deg_lon(0.0) - FDT_M_PER_DEG_LAT) < 1e-6);
    assert(fdt_geo_m_per_deg_lon(90.0) < 1e-6);
    assert(fabs(fdt_geo_m_per_deg_lon(-30.0) -
                fdt_geo_m_per_deg_lon(30.0)) < 1e-9);
}

/** Signs point the way the names say. */
static void test_directions(void)
{
    double east = 0.0;
    double north = 0.0;

    fdt_geo_offset(REF_LAT, REF_LON, REF_LAT + 0.01, REF_LON + 0.01,
                   &east, &north);
    assert(east > 0.0 && north > 0.0);

    fdt_geo_offset(REF_LAT, REF_LON, REF_LAT - 0.01, REF_LON - 0.01,
                   &east, &north);
    assert(east < 0.0 && north < 0.0);
}

/**
 * The motion the injector actually produces stays inside the world.
 *
 * tools/injector/injector.c sweeps the position by a thousandth of a degree
 * around the reference. That has to land well inside the water box of
 * adapters/webots/worlds/, which is 1000 m square, or the hulls leave the
 * scene again.
 */
static void test_injector_track_fits_the_world(void)
{
    const double sweep_deg = 0.001;
    const double water_half_m = 500.0;

    double worst = 0.0;
    for (int i = 0; i < 360; i++) {
        const double a = (double)i * M_PI / 180.0;
        double east = 0.0;
        double north = 0.0;

        fdt_geo_offset(REF_LAT, REF_LON,
                       REF_LAT + sweep_deg * sin(a),
                       REF_LON + sweep_deg * cos(a),
                       &east, &north);

        const double r = sqrt(east * east + north * north);
        if (r > worst) {
            worst = r;
        }
    }

    printf("injector track radius: %.1f m, water half-width %.0f m\n",
           worst, water_half_m);
    assert(worst < water_half_m);

    /* And it is not so small that the motion is invisible either. */
    assert(worst > 50.0);
}

/** Every output pointer is optional. */
static void test_null_outputs(void)
{
    fdt_geo_offset(REF_LAT, REF_LON, 0.0, 0.0, NULL, NULL);
}

int main(void)
{
    test_reference_is_the_origin();
    test_scales();
    test_directions();
    test_injector_track_fits_the_world();
    test_null_outputs();

    printf("test_geo: ok\n");
    return 0;
}
