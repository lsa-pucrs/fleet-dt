#ifndef FLEET_DT_DTE_H
#define FLEET_DT_DTE_H

#include <stddef.h>

/**
 * @file dte.h
 * @brief The Digital Twin Environment: parallel simulations on one tick.
 *
 * @note The boundary is deliberate and worth stating plainly. Section VI
 *       lists as future work "developing a more comprehensive model for
 *       describing vessels and fleets... That would be adequate to describe
 *       other simulations (mainly fluid and ML) in the model, at least to
 *       synchronize them with the DTI pace (simulation tick)." The DTE
 *       therefore synchronises those simulations; the formal model of
 *       Section IV does not describe them. That model belongs to the paper's
 *       roadmap, item D2 in docs/claim-map.md.
 */

/**
 * @brief One simulation's per-tick entry point.
 * @param tick  The shared tick index, starting at zero.
 * @param user  The pointer registered alongside the callback.
 */
typedef void (*fdt_sim_fn)(size_t tick, void *user);

/** One registry entry. Caller-owned, like every array in this library. */
typedef struct {
    const char *name; /**< Label, for reports. Must outlive the registry. */
    fdt_sim_fn  fn;   /**< Per-tick entry point. */
    void       *user; /**< Passed back untouched. */
} fdt_sim_slot_t;

/** Most simulations the parallel mode runs, one thread each. */
#define FDT_DTE_MAX_PARALLEL 16

/**
 * Bytes reserved inside fdt_dte_t for the threads and barriers of the
 * parallel mode. Opaque here, so this header needs no POSIX feature macro;
 * src/dte.c asserts at compile time that its bookkeeping fits.
 */
#define FDT_DTE_SYNC_BYTES 2048

/** The DTE: a fixed set of simulations sharing one clock. */
typedef struct {
    fdt_sim_slot_t *slots; /**< Caller-owned array of @c cap entries. */
    size_t          cap;   /**< Capacity. */
    size_t          n;     /**< Registered simulations. */
    size_t          tick;  /**< Ticks issued so far. */

    /* Parallel mode (Section I, contribution ii): one thread per simulation,
     * released together at the tick edge and joined before it ends. */
    int             parallel;
    _Alignas(16) unsigned char sync[FDT_DTE_SYNC_BYTES];
} fdt_dte_t;

/**
 * @brief Binds a DTE to caller-owned slot storage.
 * @return 0 on success, -1 when @p slots is NULL or @p cap is zero.
 */
int fdt_dte_init(fdt_dte_t *dte, fdt_sim_slot_t *slots, size_t cap);

/**
 * @brief Registers one simulation.
 * @param name  Label; must outlive the registry.
 * @param fn    Per-tick entry point; must not be NULL.
 * @param user  Context passed back on every tick.
 * @return 0 on success, -1 when an argument is NULL or the registry is full.
 */
int fdt_dte_register(fdt_dte_t *dte, const char *name, fdt_sim_fn fn,
                     void *user);

/**
 * @brief Runs every registered simulation on its own thread from now on.
 *
 * Section I lists as contribution (ii) that the DTE "allows running other
 * simulations in parallel in the same DTE". After this call, each
 * fdt_dte_tick() releases all simulations at once and returns when the last
 * one has finished the tick, so the tick stays the synchronisation point
 * Section VI asks for. Registration is closed once the mode starts.
 *
 * @return 0, or -1 when @p dte is NULL, already parallel, empty, holds more
 *         than FDT_DTE_MAX_PARALLEL simulations, or a thread fails to start.
 */
int fdt_dte_start_parallel(fdt_dte_t *dte);

/**
 * @brief Stops and joins the threads; later ticks run in sequence again.
 * @return 0, or -1 when @p dte is NULL or not parallel.
 */
int fdt_dte_stop_parallel(fdt_dte_t *dte);

/** @brief 1 while the parallel mode runs. */
int fdt_dte_is_parallel(const fdt_dte_t *dte);

/**
 * @brief Advances every registered simulation by one tick.
 *
 * In sequence by default; all at once after fdt_dte_start_parallel(). Either
 * way every simulation sees the same tick index and has finished it when
 * this returns.
 *
 * @return The tick index just issued, or (size_t)-1 when @p dte is NULL.
 */
size_t fdt_dte_tick(fdt_dte_t *dte);

/** @brief Registered simulations. */
size_t fdt_dte_count(const fdt_dte_t *dte);

/** @brief Ticks issued so far. */
size_t fdt_dte_ticks(const fdt_dte_t *dte);

/**
 * @brief The name of a registered simulation.
 * @return The label, or NULL when @p i is out of range.
 */
const char *fdt_dte_name(const fdt_dte_t *dte, size_t i);

#endif /* FLEET_DT_DTE_H */
