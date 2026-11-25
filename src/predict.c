#include <fleet_dt/predict.h>

#include <fleet_dt/transition.h>

#include <math.h>
#include <stddef.h>

/** Fields of B^t, in declaration order. */
#define NFIELDS 12

/** Which fields are angles in degrees, wrapped to (-180, 180]. */
static const int ANGLE[NFIELDS] = { 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0 };

static float get(const fdt_state_t *s, int f)
{
    const float v[NFIELDS] = {
        s->lat_deg, s->lon_deg, s->alt_m,
        s->roll_deg, s->pitch_deg, s->yaw_deg,
        s->surge_mps, s->sway_mps, s->heave_mps,
        s->roll_rate_rps, s->pitch_rate_rps, s->yaw_rate_rps,
    };
    return v[f];
}

static void set(fdt_state_t *s, int f, float x)
{
    float *v[NFIELDS] = {
        &s->lat_deg, &s->lon_deg, &s->alt_m,
        &s->roll_deg, &s->pitch_deg, &s->yaw_deg,
        &s->surge_mps, &s->sway_mps, &s->heave_mps,
        &s->roll_rate_rps, &s->pitch_rate_rps, &s->yaw_rate_rps,
    };
    *v[f] = x;
}

/** Wraps degrees to (-180, 180]. */
static double wrap180(double a)
{
    a = fmod(a + 180.0, 360.0);
    if (a <= 0.0) {
        a += 360.0;
    }
    return a - 180.0;
}

/**
 * Solves the normal equations of a degree-@p p least-squares fit of y over
 * t, and returns the polynomial at @p at. Gaussian elimination on at most a
 * 3x3 system, in double.
 */
static double fit_eval(const double *t, const double *y, size_t n,
                       unsigned p, double at)
{
    const unsigned m = p + 1;
    double a[3][4] = {{0}};

    for (size_t i = 0; i < n; i++) {
        double pw[5] = { 1.0, t[i], t[i] * t[i], 0.0, 0.0 };
        pw[3] = pw[2] * t[i];
        pw[4] = pw[2] * pw[2];
        for (unsigned r = 0; r < m; r++) {
            for (unsigned c = 0; c < m; c++) {
                a[r][c] += pw[r + c];
            }
            a[r][m] += pw[r] * y[i];
        }
    }

    for (unsigned c = 0; c < m; c++) {
        unsigned piv = c;
        for (unsigned r = c + 1; r < m; r++) {
            if (fabs(a[r][c]) > fabs(a[piv][c])) {
                piv = r;
            }
        }
        for (unsigned k = 0; k <= m; k++) {
            const double tmp = a[c][k];
            a[c][k] = a[piv][k];
            a[piv][k] = tmp;
        }
        for (unsigned r = 0; r < m; r++) {
            if (r == c) {
                continue;
            }
            const double f = a[r][c] / a[c][c];
            for (unsigned k = c; k <= m; k++) {
                a[r][k] -= f * a[c][k];
            }
        }
    }

    double v = 0.0;
    double pw = 1.0;
    for (unsigned c = 0; c < m; c++) {
        v += (a[c][m] / a[c][c]) * pw;
        pw *= at;
    }
    return v;
}

int fdt_predict(const fdt_queue_t *q, size_t n, unsigned d, unsigned degree,
                fdt_state_t *out)
{
    if (q == NULL || out == NULL || degree > 2 || n < degree + 1 ||
        n > FDT_PREDICT_MAX_N || fdt_window_at(q, n, n - 1) == NULL) {
        return -1;
    }

    /* Time in ticks: the newest state at 0, the oldest at -(n-1). */
    double t[FDT_PREDICT_MAX_N];
    double y[FDT_PREDICT_MAX_N];
    for (size_t k = 0; k < n; k++) {
        t[k] = -(double)k;
    }

    const fdt_state_t *newest = fdt_window_at(q, n, 0);
    fdt_state_t est = *newest;

    for (int f = 0; f < NFIELDS; f++) {
        /* Offsets from the newest sample keep float precision on lat/lon. */
        const double ref = (double)get(newest, f);
        double prev = 0.0;
        for (size_t k = 0; k < n; k++) {
            double v = (double)get(fdt_window_at(q, n, k), f) - ref;
            if (ANGLE[f]) {
                /* Unwrap against the sample one tick newer. */
                v = prev + wrap180(v - prev);
                prev = v;
            }
            y[k] = v;
        }

        double v = ref + fit_eval(t, y, n, degree, (double)d);
        if (ANGLE[f]) {
            v = wrap180(v);
        }
        set(&est, f, (float)v);
    }

    *out = est;
    return 0;
}
