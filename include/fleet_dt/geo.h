#ifndef FLEET_DT_GEO_H
#define FLEET_DT_GEO_H

/**
 * @file geo.h
 * @brief Geodetic position onto a local tangent plane.
 *
 * The twin's state carries latitude and longitude, because Table I does. A
 * renderer, a plotter or a station-keeping policy wants metres. Converting
 * between them is one subtraction and one multiplication, and getting the
 * order wrong is not obvious from reading it.
 *
 * This lives in the library rather than in the WeBots controller because that
 * is exactly where it was, and where it was wrong. The controller multiplied
 * the *absolute* longitude by metres per degree and wrote the product as a
 * world coordinate, which placed a vessel sampling the Jundiá lagoons — 30
 * degrees south, 51 degrees west — some 5700 km from the origin. The water in
 * that world is a thousand metres square, so every hull left the scene on the
 * first frame and the operator saw an empty lagoon.
 *
 * The bug survived review, a syntax check and a 674360-frame run because it
 * sat in the one file no test could reach, and because nothing about a boat
 * being absent from a 3D view makes a counter go red. Moving the arithmetic
 * here is the fix for that, not only for the arithmetic.
 */

/** Metres per degree of latitude, near enough everywhere on the ellipsoid. */
#define FDT_M_PER_DEG_LAT 111320.0

/**
 * @brief Metres per degree of longitude at a given latitude.
 * @param lat  Latitude in degrees.
 * @return The scale, which falls to zero at the poles.
 */
double fdt_geo_m_per_deg_lon(double lat);

/**
 * @brief Displacement of a point from a reference point, in metres.
 *
 * @param ref_lat  Latitude of the local origin, degrees.
 * @param ref_lon  Longitude of the local origin, degrees.
 * @param lat      Latitude of the point, degrees.
 * @param lon      Longitude of the point, degrees.
 * @param east_m   Receives metres east of the origin; may be NULL.
 * @param north_m  Receives metres north of the origin; may be NULL.
 *
 * A degree of longitude shrinks with the cosine of the latitude. At 30 degrees
 * south that is a factor of 0.87, enough to skew a track visibly, so it is
 * applied rather than assumed away.
 *
 * The result is always a displacement, never an absolute position. A caller
 * that wants world coordinates adds it to wherever the origin sits.
 */
void fdt_geo_offset(double ref_lat, double ref_lon, double lat, double lon,
                    double *east_m, double *north_m);

#endif /* FLEET_DT_GEO_H */
