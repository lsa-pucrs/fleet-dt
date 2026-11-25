#ifndef FLEET_DT_PREDICT_H
#define FLEET_DT_PREDICT_H

#include <fleet_dt/model.h>
#include <fleet_dt/queue.h>

#include <stddef.h>

/**
 * @file predict.h
 * @brief The time-series estimator of the state window.
 *
 * Section IV: the window of states "enabl[es] the implementation of
 * estimating techniques based on temporal series instead of estimators such
 * as unscented and extended Kalman filters". The talk (backup slide 6) draws
 * it: a command computed at tick k reaches the boat d >= 1 ticks later, so
 * the twin estimates
 *
 *     x̂_{k+d} = f(x_k, x_{k-1}, ..., x_{k-n+1})
 *
 * and returns the estimate to the update, as in model predictive control.
 *
 * Here f is a least-squares polynomial of degree 0, 1 or 2, fitted per field
 * of B^t over the n newest states (one tick apart) and evaluated d ticks
 * ahead. Degree 1 extrapolates the trend; degree 2 also its curvature.
 * Angles are unwrapped across +-180 degrees before the fit and wrapped back
 * after it. Latitude and longitude are fitted as offsets from the newest
 * state, so float precision holds at the scale of a hull.
 */

/** Largest window the estimator accepts. */
#define FDT_PREDICT_MAX_N 64

/**
 * @brief Estimates the state @p d ticks after the newest one in the window.
 * @param q       The twin's state queue.
 * @param n       Window depth: the n newest states feed the fit.
 * @param d       Horizon in ticks; 0 returns the fitted present.
 * @param degree  0 (mean), 1 (linear) or 2 (quadratic).
 * @param out     The estimate x̂_{k+d}.
 * @return 0 on success; -1 on a NULL argument, a degree above 2, a window
 *         shallower than degree + 1 or deeper than FDT_PREDICT_MAX_N, or a
 *         queue holding fewer than @p n states.
 */
int fdt_predict(const fdt_queue_t *q, size_t n, unsigned d, unsigned degree,
                fdt_state_t *out);

#endif /* FLEET_DT_PREDICT_H */
