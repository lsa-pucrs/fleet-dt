#ifndef FLEET_DT_WEBOTS_FLEET_H
#define FLEET_DT_WEBOTS_FLEET_H

/**
 * @file fdt_webots_fleet.h
 * @brief The vessel naming rule shared by the WeBots controller, the world
 *        generator and the world test.
 *
 * The controller resolves vessels by DEF name, in index order, and stops at
 * the first name the world does not hold. Vessels 0 to 7 carry the names of
 * the Jundiá boats; from vessel 8 on the name is BOAT<k>, two digits at least.
 * A world holding vessels 0 to N-1 therefore yields exactly N twins.
 *
 * No WeBots header is needed here, so the test builds without the SDK.
 */

#include <stddef.h>
#include <stdio.h>

/** Vessels one controller drives; slide 14 sweeps 0 to 60 boats. */
#define FDT_WEBOTS_MAX_VESSELS 64

/** The named boats, in the order the controller resolves them. */
#define FDT_WEBOTS_NAMED 8

static const char *const FDT_WEBOTS_DEF[FDT_WEBOTS_NAMED] = {
    "PINTADO", "TILAPIA", "DOURADO", "PIAVA",
    "TRAIRA",  "LAMBARI", "CARPA",   "PACU",
};

/**
 * @brief Writes the DEF name of vessel @p k into @p buf.
 * @return 0 on success, -1 when @p buf is too small.
 */
static inline int fdt_webots_vessel_def(size_t k, char *buf, size_t cap)
{
    const int n = (k < FDT_WEBOTS_NAMED)
        ? snprintf(buf, cap, "%s", FDT_WEBOTS_DEF[k])
        : snprintf(buf, cap, "BOAT%02zu", k);
    return (n > 0 && (size_t)n < cap) ? 0 : -1;
}

#endif /* FLEET_DT_WEBOTS_FLEET_H */
