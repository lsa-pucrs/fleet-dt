#ifndef FLEET_DT_GEO_H
#define FLEET_DT_GEO_H

/**
 * @file geo.h
 * @brief Geodetic position onto a local tangent plane.
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
 */
void fdt_geo_offset(double ref_lat, double ref_lon, double lat, double lon,
                    double *east_m, double *north_m);

#endif /* FLEET_DT_GEO_H */
