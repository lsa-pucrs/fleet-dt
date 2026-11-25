#ifndef FLEET_DT_WORLDGEN_H
#define FLEET_DT_WORLDGEN_H

#include <stddef.h>

/**
 * @file worldgen.h
 * @brief Builds an N-vessel WeBots world from worlds/jundia_fleet.wbt.
 *
 * Slide 14 sweeps the simulation from 0 to 60 boats. The repository commits
 * worlds with 0, 1 and 2 hulls; this generator writes the rest, so every
 * world in the sweep shares the water, the light, the obstacles and the hull
 * of the committed fleet world, and differs from it only in vessel count.
 *
 * Vessels 0 and 1 are copied unchanged, so the two-vessel output equals the
 * hull section of jundia_fleet.wbt. Vessel k >= 2 is a copy of vessel 1 under
 * the DEF name of fdt_webots_fleet.h, on a grid 4 m apart, north of the
 * obstacles. Only vessel 0 carries the controller.
 */

/** Metres between neighbouring hulls of the generated grid. */
#define FDT_WBTGEN_SPACING_M 4.0

/** Hulls per grid row. */
#define FDT_WBTGEN_COLUMNS 8

/**
 * @brief Writes the world with @p n vessels.
 * @param tmpl  The text of jundia_fleet.wbt, NUL-terminated.
 * @param n     Vessels, 1 to FDT_WEBOTS_MAX_VESSELS.
 * @param out   Destination; NUL-terminated on success.
 * @return Bytes written, excluding the NUL, or -1 when @p tmpl lacks the
 *         vessel markers, @p n is out of range, or @p cap is too small.
 */
long fdt_wbtgen(const char *tmpl, unsigned n, char *out, size_t cap);

#endif /* FLEET_DT_WORLDGEN_H */
