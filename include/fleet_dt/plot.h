#ifndef FLEET_DT_PLOT_H
#define FLEET_DT_PLOT_H

#include <stddef.h>

/**
 * @file plot.h
 * @brief A dependency-free SVG plotter for the measurement artefacts.
 */

/** Maximum series in one chart. */
#define FDT_PLOT_MAX_SERIES 6

/** Maximum reference lines in one chart. */
#define FDT_PLOT_MAX_RULES 4

/** How a series is drawn. */
typedef enum {
    FDT_PLOT_LINE,    /**< Polyline through the points. */
    FDT_PLOT_BAR,     /**< Vertical bars from the x axis. */
    FDT_PLOT_SCATTER  /**< Unconnected markers. */
} fdt_plot_kind_t;

/** One series: caller-owned parallel arrays of x and y. */
typedef struct {
    const char     *name; /**< Legend label. Never NULL. */
    const double   *x;    /**< Abscissae, @c n entries. */
    const double   *y;    /**< Ordinates, @c n entries. */
    size_t          n;    /**< Points in this series. */
    fdt_plot_kind_t kind; /**< How to draw it. */
} fdt_plot_series_t;

/**
 * A horizontal reference line.
 */
typedef struct {
    double      y;     /**< Ordinate of the rule. */
    const char *label; /**< Text drawn beside it, e.g. the paper's figure. */
} fdt_plot_rule_t;

/** A chart. Treat the fields as private; use the functions. */
typedef struct {
    const char *title;   /**< Chart title. */
    const char *x_label; /**< Abscissa label. */
    const char *y_label; /**< Ordinate label. */
    int         width;   /**< Viewport width in px; default 860. */
    int         height;  /**< Viewport height in px; default 420. */
    int         log_y;   /**< Non-zero to plot the ordinate logarithmically. */

    fdt_plot_series_t series[FDT_PLOT_MAX_SERIES];
    size_t            nseries;

    fdt_plot_rule_t rules[FDT_PLOT_MAX_RULES];
    size_t          nrules;
} fdt_plot_t;

/**
 * @brief Initialises a chart with the default geometry.
 * @param title    Chart title, must outlive the plot.
 * @param x_label  Abscissa label, must outlive the plot.
 * @param y_label  Ordinate label, must outlive the plot.
 * @return 0 on success, -1 when @p p is NULL.
 */
int fdt_plot_init(fdt_plot_t *p, const char *title,
                  const char *x_label, const char *y_label);

/**
 * @brief Appends a series.
 * @param name  Legend label; must outlive the plot.
 * @param x     Abscissae, @p n entries, caller-owned, must outlive the plot.
 * @param y     Ordinates, @p n entries, same lifetime.
 * @param n     Point count; zero is rejected.
 * @return 0 on success, -1 when an argument is NULL, @p n is zero, or the
 *         chart already holds ::FDT_PLOT_MAX_SERIES series.
 */
int fdt_plot_series(fdt_plot_t *p, const char *name,
                    const double *x, const double *y, size_t n,
                    fdt_plot_kind_t kind);

/**
 * @brief Appends a horizontal reference line, typically a published figure.
 * @param y      Ordinate of the rule.
 * @param label  Text drawn beside it; must outlive the plot.
 * @return 0 on success, -1 when full or when @p p is NULL.
 */
int fdt_plot_hline(fdt_plot_t *p, double y, const char *label);

/**
 * @brief Switches the ordinate to a base-10 logarithmic scale.
 */
void fdt_plot_set_log_y(fdt_plot_t *p, int on);

/**
 * @brief Renders the chart to an SVG file, creating parent directories.
 * @param path  Destination, e.g. "results/jitter.svg".
 * @return 0 on success, -1 when the chart is empty or the file cannot be
 *         written.
 */
int fdt_plot_write_svg(const fdt_plot_t *p, const char *path);

/**
 * @brief Writes the chart's series as CSV, one column per series.
 *
 * @return 0 on success, -1 when the chart is empty or the file cannot be
 *         written.
 */
int fdt_plot_write_csv(const fdt_plot_t *p, const char *path);

/**
 * @brief Creates a directory if it does not already exist.
 * @return 0 when the directory exists on return, -1 otherwise.
 */
int fdt_mkdir_p(const char *path);

#endif /* FLEET_DT_PLOT_H */
