#include <fleet_dt/geo.h>

#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double fdt_geo_m_per_deg_lon(double lat)
{
    return FDT_M_PER_DEG_LAT * cos(lat * M_PI / 180.0);
}

void fdt_geo_offset(double ref_lat, double ref_lon, double lat, double lon,
                    double *east_m, double *north_m)
{
    /* The subtraction comes first. Scaling an absolute coordinate and calling
     * the product a position is precisely the mistake this function exists to
     * make impossible. */
    if (east_m != NULL) {
        *east_m = (lon - ref_lon) * fdt_geo_m_per_deg_lon(ref_lat);
    }
    if (north_m != NULL) {
        *north_m = (lat - ref_lat) * FDT_M_PER_DEG_LAT;
    }
}
