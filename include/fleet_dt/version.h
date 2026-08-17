#ifndef FLEET_DT_VERSION_H
#define FLEET_DT_VERSION_H

/**
 * @file version.h
 * @brief Library version and the manuscript revision this code tracks.
 */

/** Manuscript revision tracked by this repository: model in Section IV,
 *  equations (1) through (6). */
#define FDT_PAPER_REVISION "-10"

/** Library version, semantic versioning. */
#define FDT_VERSION "0.1.0"

/**
 * @brief Human-readable version banner.
 * @return A static string of the form
 *         "fleet-dt 0.1.0 (ICECS 2026 manuscript -10)". Never NULL, never
 *         owned by the caller.
 */
const char *fdt_version(void);

#endif /* FLEET_DT_VERSION_H */
