/**
 * @file test_fleet_window.c
 * @brief Equation (6), F^t = Delta^e([F^{t-1}; F^{t-n}], I^{t-1}), with the
 *        c^t Section IV gives as its example: inter-vessel distance.
 */
#include <fleet_dt/coordinator.h>
#include <fleet_dt/geo.h>
#include <fleet_dt/tick.h>

#include <assert.h>
#include <math.h>
#include <stdio.h>

#define BOATS  2
#define CAP    16
#define WINDOW 4
#define DT_S   ((double)FDT_TICK_NS / 1e9)

/** c^t: spacing now, and how fast it shrinks across the window. */
typedef struct {
    double min_m;
    double closing_mps;
} spacing_ctx_t;

static fdt_state_t   g_q[BOATS][CAP];
static fdt_twin_t    g_tw[BOATS];
static fdt_state_t   g_slots[BOATS];
static spacing_ctx_t g_hist[CAP];

/** delta^e: move at surge along yaw (north = 0 deg, east = 90 deg). */
static void move(const fdt_queue_t *q, size_t n, const fdt_input_t *in,
                 const fdt_goal_t *g_prev, fdt_state_t *out, void *ctx,
                 void *fleet_ctx)
{
    (void)in;
    (void)g_prev;
    (void)ctx;
    (void)fleet_ctx;
    *out = *fdt_window_at(q, n, 0);
    const double d = (double)out->surge_mps * DT_S;
    const double yaw = (double)out->yaw_deg * 3.14159265358979 / 180.0;
    out->lat_deg += (float)(d * cos(yaw) / FDT_M_PER_DEG_LAT);
    out->lon_deg += (float)(d * sin(yaw) /
                            fdt_geo_m_per_deg_lon((double)out->lat_deg));
}

static void hold(const fdt_state_t *b, const fdt_goal_t *g, fdt_actuation_t *a,
                 void *ctx, void *fleet_ctx)
{
    (void)b;
    (void)g;
    (void)ctx;
    (void)fleet_ctx;
    *a = (fdt_actuation_t){ .throttle_pct = 50.0f, .cage_rad = 0.0f };
}

static void plain_ctx(const fdt_store_t *bt, void *fc, void *user)
{
    (void)bt;
    (void)fc;
    (void)user;
}

static void plan(const fdt_store_t *bt, const void *fc, fdt_goal_t *g,
                 size_t n, void *user)
{
    (void)bt;
    (void)fc;
    (void)g;
    (void)n;
    (void)user;
}

/**
 * c^t over the window: spacing from B^t, closing speed from the spacing
 * recorded in the oldest c^t the window reaches.
 */
static void spacing_over_window(const fdt_fleet_t *f, size_t n,
                                const fdt_store_t *bt, void *fleet_ctx,
                                void *user)
{
    (void)user;
    spacing_ctx_t *c = (spacing_ctx_t *)fleet_ctx;
    assert(fdt_store_spacing(bt, &c->min_m, NULL, NULL, NULL) == 0);

    c->closing_mps = 0.0;
    for (size_t k = n - 1; k + 1 > 0; k--) {
        const spacing_ctx_t *old = fdt_fleet_ctx_at(f, n, k);
        if (old != NULL) {
            /* old is c^{t-1-k}: k + 1 ticks before this frame. */
            c->closing_mps = (old->min_m - c->min_m) /
                             ((double)(k + 1) * DT_S);
            break;
        }
    }
}

int main(void)
{
    /* Two boats 20 m apart on a north-south line, heading at each other
     * at 1 m/s each. B^t holds latitude as a float (the 48-byte state of
     * Section IV): near -30 degrees one float step is about 0.21 m, more
     * than the 0.125 m a boat moves per tick, so a delta^e that integrates
     * position in B^t itself rounds every step. This test sails near the
     * equator, where the float step is millimetres; a real delta^e keeps
     * its position in double in its own context. */
    const double lat0 = 0.001, lon0 = 0.001;
    for (int k = 0; k < BOATS; k++) {
        assert(fdt_twin_init(&g_tw[k], g_q[k], CAP, move, hold, NULL) == 0);
        fdt_state_t b = {0};
        b.lat_deg   = (float)(lat0 + (k == 0 ? 0.0 : 20.0) / FDT_M_PER_DEG_LAT);
        b.lon_deg   = (float)lon0;
        b.yaw_deg   = (k == 0) ? 0.0f : 180.0f;
        b.surge_mps = 1.0f;
        for (int s = 0; s < WINDOW; s++) {
            assert(fdt_twin_seed(&g_tw[k], &b) == 0);
        }
    }

    static spacing_ctx_t ctx;
    fdt_fleet_t fleet;
    fdt_store_t store;
    fdt_coord_t co;
    assert(fdt_fleet_init(&fleet, g_tw, BOATS, &ctx) == 0);
    assert(fdt_fleet_keep_history(&fleet, NULL, CAP, sizeof ctx) == -1);
    assert(fdt_fleet_keep_history(&fleet, g_hist, CAP, 0) == -1);
    assert(fdt_fleet_record_ctx(&fleet) == -1); /* no history yet */
    assert(fdt_fleet_keep_history(&fleet, g_hist, CAP, sizeof ctx) == 0);
    assert(fdt_store_init(&store, g_slots, BOATS) == 0);
    assert(fdt_coord_init(&co, &fleet, &store, plain_ctx, plan, NULL) == 0);
    assert(fdt_coord_set_ctx_window(&co, NULL) == -1);
    assert(fdt_coord_set_ctx_window(&co, spacing_over_window) == 0);

    assert(fdt_fleet_ctx_at(&fleet, WINDOW, 0) == NULL); /* empty history */

    fdt_input_t     ins[BOATS] = {{0}};
    fdt_goal_t      gp[BOATS] = {{0}}, gn[BOATS] = {{0}};
    fdt_state_t     bs[BOATS];
    fdt_actuation_t as[BOATS];

    for (int t = 0; t < 24; t++) {
        assert(fdt_coord_step(&co, ins, WINDOW, gp, gn, bs, as) == 0);
    }

    /* 24 ticks of 0.125 s at a 2 m/s closing speed: 6 m closed. */
    printf("after 3 s: spacing %.2f m, closing %.2f m/s over the window\n",
           ctx.min_m, ctx.closing_mps);
    assert(fabs(ctx.min_m - 14.0) < 0.05);
    assert(fabs(ctx.closing_mps - 2.0) < 0.05);

    /* The window reads back what the frames recorded, newest first. */
    const spacing_ctx_t *c0 = fdt_fleet_ctx_at(&fleet, WINDOW, 0);
    const spacing_ctx_t *c3 = fdt_fleet_ctx_at(&fleet, WINDOW, 3);
    assert(c0 != NULL && c3 != NULL);
    assert(fabs(c0->min_m - ctx.min_m) < 1e-9); /* c^t just recorded */
    assert(fabs((c3->min_m - c0->min_m) - 3 * 2.0 * DT_S) < 0.02);
    assert(fdt_fleet_ctx_at(&fleet, WINDOW, WINDOW) == NULL);

    const fdt_state_t *b0 = fdt_fleet_state_at(&fleet, WINDOW, 0, 1);
    const fdt_state_t *b3 = fdt_fleet_state_at(&fleet, WINDOW, 3, 1);
    assert(b0 != NULL && b3 != NULL);
    assert(b0->lat_deg < b3->lat_deg); /* boat 1 heads south */
    assert(fdt_fleet_state_at(&fleet, WINDOW, 0, BOATS) == NULL);
    assert(fdt_fleet_state_at(&fleet, WINDOW, WINDOW, 0) == NULL);

    /* Spacing alone, on a store of three. */
    fdt_state_t three[3] = {{0}};
    three[1].lat_deg = (float)(5.0 / FDT_M_PER_DEG_LAT);
    three[2].lat_deg = (float)(12.0 / FDT_M_PER_DEG_LAT);
    fdt_store_t st3 = { three, 3 };
    double lo = 0.0, mean = 0.0;
    size_t a = 9, b = 9;
    assert(fdt_store_spacing(&st3, &lo, &mean, &a, &b) == 0);
    assert(fabs(lo - 5.0) < 0.01 && a == 0 && b == 1);
    assert(fabs(mean - (5.0 + 12.0 + 7.0) / 3.0) < 0.01);
    fdt_store_t st1 = { three, 1 };
    assert(fdt_store_spacing(&st1, &lo, NULL, NULL, NULL) == -1);

    printf("test_fleet_window: ok\n");
    return 0;
}
