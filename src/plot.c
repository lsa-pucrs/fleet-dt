#define _POSIX_C_SOURCE 200809L

#include <fleet_dt/plot.h>

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#define MARGIN_LEFT   72
#define MARGIN_RIGHT  190
#define MARGIN_TOP    52
#define MARGIN_BOTTOM 62
#define TICKS_X       6
#define TICKS_Y       5

/**
 * Series colours.
 */
static const char *const SERIES_COLOR[FDT_PLOT_MAX_SERIES] = {
    "#2563eb", /* blue   */
    "#dc2626", /* red    */
    "#059669", /* green  */
    "#d97706", /* amber  */
    "#7c3aed", /* violet */
    "#0891b2", /* cyan   */
};

int fdt_mkdir_p(const char *path)
{
    if (path == NULL || path[0] == '\0') {
        return -1;
    }
    if (mkdir(path, 0755) == 0 || errno == EEXIST) {
        return 0;
    }
    return -1;
}

int fdt_plot_init(fdt_plot_t *p, const char *title,
                  const char *x_label, const char *y_label)
{
    if (p == NULL) {
        return -1;
    }
    memset(p, 0, sizeof *p);
    p->title   = (title != NULL) ? title : "";
    p->x_label = (x_label != NULL) ? x_label : "";
    p->y_label = (y_label != NULL) ? y_label : "";
    p->width   = 860;
    p->height  = 420;
    p->log_y   = 0;
    return 0;
}

int fdt_plot_series(fdt_plot_t *p, const char *name,
                    const double *x, const double *y, size_t n,
                    fdt_plot_kind_t kind)
{
    if (p == NULL || name == NULL || x == NULL || y == NULL || n == 0) {
        return -1;
    }
    if (p->nseries >= FDT_PLOT_MAX_SERIES) {
        return -1;
    }
    p->series[p->nseries].name = name;
    p->series[p->nseries].x    = x;
    p->series[p->nseries].y    = y;
    p->series[p->nseries].n    = n;
    p->series[p->nseries].kind = kind;
    p->nseries++;
    return 0;
}

int fdt_plot_hline(fdt_plot_t *p, double y, const char *label)
{
    if (p == NULL || p->nrules >= FDT_PLOT_MAX_RULES) {
        return -1;
    }
    p->rules[p->nrules].y     = y;
    p->rules[p->nrules].label = (label != NULL) ? label : "";
    p->nrules++;
    return 0;
}

void fdt_plot_set_log_y(fdt_plot_t *p, int on)
{
    if (p != NULL) {
        p->log_y = on;
    }
}

/** Data ranges over every series, widened to include the reference lines. */
typedef struct {
    double x_min, x_max;
    double y_min, y_max;
    double y_pos_min; /**< Smallest strictly positive y, for the log axis. */
} bounds_t;

/**
 * @brief A tick step a reader can hold in their head.
 *
 * Dividing a range into a fixed number of equal parts produces labels like
 * 107.1 and 0.8333, which say nothing about the data and everything about the
 * divisor. This rounds the step to 1, 2, 2.5 or 5 times a power of ten, so the
 * labels land on values a reader recognises.
 */
static double nice_step(double range, int target)
{
    double raw, mag, frac;

    if (!(range > 0.0) || target <= 0) {
        return 1.0;
    }
    raw  = range / (double)target;
    mag  = pow(10.0, floor(log10(raw)));
    frac = raw / mag;

    if (frac <= 1.0) {
        return 1.0 * mag;
    }
    if (frac <= 2.0) {
        return 2.0 * mag;
    }
    if (frac <= 2.5) {
        return 2.5 * mag;
    }
    if (frac <= 5.0) {
        return 5.0 * mag;
    }
    return 10.0 * mag;
}

/** Non-zero when every abscissa of every series is a whole number. */
static int x_is_integral(const fdt_plot_t *p)
{
    for (size_t s = 0; s < p->nseries; s++) {
        for (size_t i = 0; i < p->series[s].n; i++) {
            const double v = p->series[s].x[i];

            if (v != floor(v) || fabs(v) > 1e9) {
                return 0;
            }
        }
    }
    return 1;
}

/** Smallest gap between neighbouring abscissae of the bar series. */
static double min_bar_gap(const fdt_plot_t *p)
{
    double gap = 0.0;

    for (size_t s = 0; s < p->nseries; s++) {
        if (p->series[s].kind != FDT_PLOT_BAR) {
            continue;
        }
        for (size_t i = 1; i < p->series[s].n; i++) {
            const double d = fabs(p->series[s].x[i] - p->series[s].x[i - 1]);

            if (d > 0.0 && (gap == 0.0 || d < gap)) {
                gap = d;
            }
        }
    }
    return gap;
}

static bounds_t compute_bounds(const fdt_plot_t *p)
{
    bounds_t b = { .x_min = 1e308, .x_max = -1e308,
                   .y_min = 1e308, .y_max = -1e308,
                   .y_pos_min = 1e308 };

    for (size_t s = 0; s < p->nseries; s++) {
        for (size_t i = 0; i < p->series[s].n; i++) {
            const double xv = p->series[s].x[i];
            const double yv = p->series[s].y[i];
            if (xv < b.x_min) { b.x_min = xv; }
            if (xv > b.x_max) { b.x_max = xv; }
            if (yv < b.y_min) { b.y_min = yv; }
            if (yv > b.y_max) { b.y_max = yv; }
            if (yv > 0.0 && yv < b.y_pos_min) { b.y_pos_min = yv; }
        }
    }

    for (size_t r = 0; r < p->nrules; r++) {
        const double yv = p->rules[r].y;
        if (yv < b.y_min) { b.y_min = yv; }
        if (yv > b.y_max) { b.y_max = yv; }
        if (yv > 0.0 && yv < b.y_pos_min) { b.y_pos_min = yv; }
    }

    for (size_t s = 0; s < p->nseries; s++) {
        if (p->series[s].kind == FDT_PLOT_BAR && b.y_min > 0.0) {
            b.y_min = 0.0;
        }
    }

    if (b.x_max <= b.x_min) { b.x_max = b.x_min + 1.0; }
    if (b.y_max <= b.y_min) { b.y_max = b.y_min + 1.0; }

    /* Headroom, so the topmost point is not welded to the frame. */
    const double pad = (b.y_max - b.y_min) * 0.08;
    b.y_max += pad;
    if (b.y_min != 0.0) {
        b.y_min -= pad;
    }
    if (b.y_pos_min > 1e307) {
        b.y_pos_min = 1e-9;
    }

    /* A bar centred on the first abscissa would hang half outside the frame,
       so the horizontal range widens by half a bar on each side. */
    {
        const double gap = min_bar_gap(p);

        if (gap > 0.0) {
            b.x_min -= gap / 2.0;
            b.x_max += gap / 2.0;
        }
    }

    /* Snap the linear ordinate to the tick step, so the axis begins and ends
       on a label rather than on a remainder. */
    if (!p->log_y) {
        const double step = nice_step(b.y_max - b.y_min, TICKS_Y);

        if (step > 0.0) {
            b.y_min = floor(b.y_min / step) * step;
            b.y_max = ceil(b.y_max / step) * step;
        }
    }
    return b;
}

/** Maps a data ordinate to a pixel row, honouring the log setting. */
static double map_y(const fdt_plot_t *p, const bounds_t *b, double v,
                    double top, double bottom)
{
    if (p->log_y) {
        const double lo = log10((b->y_min > 0.0) ? b->y_min : b->y_pos_min);
        const double hi = log10((b->y_max > 0.0) ? b->y_max : b->y_pos_min);
        const double vv = log10((v > 0.0) ? v : b->y_pos_min);
        const double t  = (hi > lo) ? (vv - lo) / (hi - lo) : 0.0;
        return bottom - t * (bottom - top);
    }
    const double t = (v - b->y_min) / (b->y_max - b->y_min);
    return bottom - t * (bottom - top);
}

/** Maps a data abscissa to a pixel column. */
static double map_x(const bounds_t *b, double v, double left, double right)
{
    const double t = (v - b->x_min) / (b->x_max - b->x_min);
    return left + t * (right - left);
}

/** Writes text with the five XML entities escaped. */
static void put_escaped(FILE *f, const char *s)
{
    for (; *s != '\0'; s++) {
        switch (*s) {
        case '&':  fputs("&amp;", f);  break;
        case '<':  fputs("&lt;", f);   break;
        case '>':  fputs("&gt;", f);   break;
        case '"':  fputs("&quot;", f); break;
        case '\'': fputs("&apos;", f); break;
        default:   fputc(*s, f);       break;
        }
    }
}

/** The stylesheet, including the dark-mode rule. */
static void put_style(FILE *f)
{
    fputs("<style>\n"
          "  .bg{fill:#ffffff}\n"
          "  .frame{stroke:#94a3b8;stroke-width:1;fill:none}\n"
          "  .grid{stroke:#e2e8f0;stroke-width:1}\n"
          "  .tick{fill:#475569;font:11px sans-serif}\n"
          "  .axis{fill:#334155;font:12px sans-serif}\n"
          "  .title{fill:#0f172a;font:bold 15px sans-serif}\n"
          "  .rule{stroke:#64748b;stroke-width:1.5;stroke-dasharray:6 4}\n"
          "  .rulelab{fill:#475569;font:italic 11px sans-serif}\n"
          "  .legend{fill:#334155;font:12px sans-serif}\n"
          "  @media (prefers-color-scheme: dark){\n"
          "    .bg{fill:#0b1120}\n"
          "    .frame{stroke:#475569}\n"
          "    .grid{stroke:#1e293b}\n"
          "    .tick{fill:#94a3b8}\n"
          "    .axis{fill:#cbd5e1}\n"
          "    .title{fill:#f1f5f9}\n"
          "    .rule{stroke:#94a3b8}\n"
          "    .rulelab{fill:#94a3b8}\n"
          "    .legend{fill:#cbd5e1}\n"
          "  }\n"
          "</style>\n", f);
}

/** Axis ticks, gridlines and their labels. */
static void put_axes(FILE *f, const fdt_plot_t *p, const bounds_t *b,
                     double left, double right, double top, double bottom)
{
    if (p->log_y) {
        for (int i = 0; i <= TICKS_Y; i++) {
            const double t = (double)i / TICKS_Y;
            const double py = bottom - t * (bottom - top);
            const double lo = log10((b->y_min > 0.0) ? b->y_min : b->y_pos_min);
            const double hi = log10((b->y_max > 0.0) ? b->y_max : b->y_pos_min);
            const double value = pow(10.0, lo + t * (hi - lo));

            fprintf(f, "<line class=\"grid\" x1=\"%.1f\" y1=\"%.1f\" "
                       "x2=\"%.1f\" y2=\"%.1f\"/>\n", left, py, right, py);
            fprintf(f, "<text class=\"tick\" x=\"%.1f\" y=\"%.1f\" "
                       "text-anchor=\"end\">%.3g</text>\n",
                    left - 8.0, py + 4.0, value);
        }
    } else {
        const double step = nice_step(b->y_max - b->y_min, TICKS_Y);
        double v;

        for (v = ceil(b->y_min / step) * step; v <= b->y_max + step * 1e-6;
             v += step) {
            const double t = (v - b->y_min) / (b->y_max - b->y_min);
            const double py = bottom - t * (bottom - top);
            const double shown = (fabs(v) < step * 1e-9) ? 0.0 : v;

            fprintf(f, "<line class=\"grid\" x1=\"%.1f\" y1=\"%.1f\" "
                       "x2=\"%.1f\" y2=\"%.1f\"/>\n", left, py, right, py);
            fprintf(f, "<text class=\"tick\" x=\"%.1f\" y=\"%.1f\" "
                       "text-anchor=\"end\">%.6g</text>\n",
                    left - 8.0, py + 4.0, shown);
        }
    }

    {
        double step = nice_step(b->x_max - b->x_min, TICKS_X);
        double v;

        /* Whole-numbered abscissae, such as a vessel count or a frame index,
           never take a fractional tick: a label of 0.8333 vessels describes
           the divisor and not the data. */
        if (x_is_integral(p) && step < 1.0) {
            step = 1.0;
        }

        for (v = ceil(b->x_min / step) * step; v <= b->x_max + step * 1e-6;
             v += step) {
            const double t = (v - b->x_min) / (b->x_max - b->x_min);
            const double px = left + t * (right - left);
            const double shown = (fabs(v) < step * 1e-9) ? 0.0 : v;

            fprintf(f, "<line class=\"grid\" x1=\"%.1f\" y1=\"%.1f\" "
                       "x2=\"%.1f\" y2=\"%.1f\"/>\n", px, top, px, bottom);
            fprintf(f, "<text class=\"tick\" x=\"%.1f\" y=\"%.1f\" "
                       "text-anchor=\"middle\">%.6g</text>\n",
                    px, bottom + 18.0, shown);
        }
    }

    fprintf(f, "<rect class=\"frame\" x=\"%.1f\" y=\"%.1f\" "
               "width=\"%.1f\" height=\"%.1f\"/>\n",
            left, top, right - left, bottom - top);

    fprintf(f, "<text class=\"axis\" x=\"%.1f\" y=\"%.1f\" "
               "text-anchor=\"middle\">", (left + right) / 2.0, bottom + 44.0);
    put_escaped(f, p->x_label);
    fputs("</text>\n", f);

    fprintf(f, "<text class=\"axis\" x=\"%.1f\" y=\"%.1f\" "
               "text-anchor=\"middle\" transform=\"rotate(-90 %.1f %.1f)\">",
            left - 50.0, (top + bottom) / 2.0,
            left - 50.0, (top + bottom) / 2.0);
    put_escaped(f, p->y_label);
    fputs("</text>\n", f);
}

/** One series, drawn according to its kind. */
static void put_series(FILE *f, const fdt_plot_t *p, const bounds_t *b,
                       size_t s, double left, double right,
                       double top, double bottom)
{
    const fdt_plot_series_t *ser = &p->series[s];
    const char *color = SERIES_COLOR[s % FDT_PLOT_MAX_SERIES];

    if (ser->kind == FDT_PLOT_LINE) {
        fprintf(f, "<polyline fill=\"none\" stroke=\"%s\" stroke-width=\"2\" "
                   "stroke-linejoin=\"round\" points=\"", color);
        for (size_t i = 0; i < ser->n; i++) {
            fprintf(f, "%.2f,%.2f ",
                    map_x(b, ser->x[i], left, right),
                    map_y(p, b, ser->y[i], top, bottom));
        }
        fputs("\"/>\n", f);
        return;
    }

    if (ser->kind == FDT_PLOT_SCATTER) {
        for (size_t i = 0; i < ser->n; i++) {
            fprintf(f, "<circle cx=\"%.2f\" cy=\"%.2f\" r=\"3\" "
                       "fill=\"%s\" fill-opacity=\"0.75\"/>\n",
                    map_x(b, ser->x[i], left, right),
                    map_y(p, b, ser->y[i], top, bottom), color);
        }
        return;
    }

    size_t bar_series = 0;
    size_t bar_index  = 0;
    for (size_t k = 0; k < p->nseries; k++) {
        if (p->series[k].kind == FDT_PLOT_BAR) {
            if (k == s) {
                bar_index = bar_series;
            }
            bar_series++;
        }
    }
    if (bar_series == 0) {
        bar_series = 1;
    }

    /* The slot follows the spacing of the data, not the number of points.
       Deriving it from the count draws every bar the same width even when the
       abscissae are unevenly spaced, which overlaps them where the data is
       dense and strands them where it is sparse. */
    const double gap  = min_bar_gap(p);
    const double span = (b->x_max > b->x_min) ? (b->x_max - b->x_min) : 1.0;
    const double slot = (gap > 0.0)
                            ? (right - left) * (gap / span)
                            : (right - left) / (double)(ser->n > 0 ? ser->n : 1);
    double w = (slot * 0.72) / (double)bar_series;

    /* Logarithmically spaced abscissae put the closest pair of points a
       thousandth of the range apart, and a bar that thin renders as nothing.
       Three pixels is the floor at which a bar still reads as a bar; below it
       the chart silently loses its data. */
    if (w < 3.0) {
        w = 3.0;
    }
    const double zero = map_y(p, b, (b->y_min < 0.0) ? 0.0 : b->y_min,
                              top, bottom);

    for (size_t i = 0; i < ser->n; i++) {
        const double cx = map_x(b, ser->x[i], left, right);
        const double py = map_y(p, b, ser->y[i], top, bottom);
        const double x0 = cx - (w * (double)bar_series) / 2.0 +
                          w * (double)bar_index;
        /* A bar grows from the baseline towards its value, in whichever
           direction that lies. Anchoring it at the value instead leaves a
           negative reading as a sliver hanging off the axis. */
        const double y0 = (py < zero) ? py : zero;
        const double h  = fabs(zero - py);

        fprintf(f, "<rect x=\"%.2f\" y=\"%.2f\" width=\"%.2f\" "
                   "height=\"%.2f\" fill=\"%s\" fill-opacity=\"0.85\"/>\n",
                x0, y0, w, (h > 1.0) ? h : 1.0, color);
    }
}

int fdt_plot_write_svg(const fdt_plot_t *p, const char *path)
{
    if (p == NULL || path == NULL || p->nseries == 0) {
        return -1;
    }

    FILE *f = fopen(path, "w");
    if (f == NULL) {
        return -1;
    }

    const double left   = MARGIN_LEFT;
    const double right  = p->width - MARGIN_RIGHT;
    const double top    = MARGIN_TOP;
    const double bottom = p->height - MARGIN_BOTTOM;
    const bounds_t b = compute_bounds(p);

    fprintf(f, "<svg xmlns=\"http://www.w3.org/2000/svg\" "
               "viewBox=\"0 0 %d %d\" width=\"%d\" height=\"%d\" "
               "role=\"img\" aria-label=\"",
            p->width, p->height, p->width, p->height);
    put_escaped(f, p->title);
    fputs("\">\n", f);

    put_style(f);
    fprintf(f, "<rect class=\"bg\" x=\"0\" y=\"0\" width=\"%d\" height=\"%d\"/>\n",
            p->width, p->height);

    fprintf(f, "<text class=\"title\" x=\"%.1f\" y=\"28\">", left);
    put_escaped(f, p->title);
    fputs("</text>\n", f);

    put_axes(f, p, &b, left, right, top, bottom);

    /* Reference lines first, so the data draws on top of them.

       The caption sits inside the plot, right-aligned above its own line. It
       used to sit in the right-hand gutter at the height of the line, which
       is where the legend lives, so any rule near the top of the range landed
       on top of a legend entry. */
    for (size_t r = 0; r < p->nrules; r++) {
        const double py = map_y(p, &b, p->rules[r].y, top, bottom);
        const double ty = (py - 6.0 < top + 12.0) ? (py + 14.0) : (py - 6.0);

        fprintf(f, "<line class=\"rule\" x1=\"%.1f\" y1=\"%.1f\" "
                   "x2=\"%.1f\" y2=\"%.1f\"/>\n", left, py, right, py);
        fprintf(f, "<text class=\"rulelab\" x=\"%.1f\" y=\"%.1f\" "
                   "text-anchor=\"end\">", right - 6.0, ty);
        put_escaped(f, p->rules[r].label);
        fputs("</text>\n", f);
    }

    for (size_t s = 0; s < p->nseries; s++) {
        put_series(f, p, &b, s, left, right, top, bottom);
    }

    /* Legend, alone in the right-hand gutter. */
    double ly = top + 12.0;
    for (size_t s = 0; s < p->nseries; s++) {
        fprintf(f, "<rect x=\"%.1f\" y=\"%.1f\" width=\"12\" height=\"12\" "
                   "fill=\"%s\"/>\n",
                right + 10.0, ly - 10.0,
                SERIES_COLOR[s % FDT_PLOT_MAX_SERIES]);
        fprintf(f, "<text class=\"legend\" x=\"%.1f\" y=\"%.1f\">",
                right + 28.0, ly);
        put_escaped(f, p->series[s].name);
        fputs("</text>\n", f);
        ly += 20.0;
    }

    fputs("</svg>\n", f);
    return (fclose(f) == 0) ? 0 : -1;
}

int fdt_plot_write_csv(const fdt_plot_t *p, const char *path)
{
    if (p == NULL || path == NULL || p->nseries == 0) {
        return -1;
    }

    FILE *f = fopen(path, "w");
    if (f == NULL) {
        return -1;
    }

    fputs("x", f);
    for (size_t s = 0; s < p->nseries; s++) {
        fputc(',', f);
        put_escaped(f, p->series[s].name);
    }
    fputc('\n', f);

    for (size_t i = 0; i < p->series[0].n; i++) {
        fprintf(f, "%.10g", p->series[0].x[i]);
        for (size_t s = 0; s < p->nseries; s++) {
            fputc(',', f);
            if (i < p->series[s].n) {
                fprintf(f, "%.10g", p->series[s].y[i]);
            }
        }
        fputc('\n', f);
    }

    return (fclose(f) == 0) ? 0 : -1;
}
