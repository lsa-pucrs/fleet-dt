/**
 * @file bench_webots_cpu.c
 * @brief The CPU cost of the simulation against vessel count: Section V-A,
 *        and slide 14 of the talk (0 to 60 boats).
 *
 * Each world of the sweep runs in WeBots for a warmup and a sample, and the
 * bench reads the CPU the WeBots process tree consumed. The synthetic boats
 * run inside the controller, so the controller times them apart
 * (FDT_CPU_REPORT) and the bench subtracts their share: what remains is the
 * simulation and the twins. A least-squares line through the worlds with at
 * least one vessel gives the cost per added twin.
 *
 * Slide 14 ran "Webots R2025a on one core, rendering off". This bench does
 * the same: it pins the tree to one core (FDT_CPU_CORE, the last online core
 * by default; -1 leaves it unpinned) and passes --no-rendering.
 */
#define _GNU_SOURCE

#include "bench_common.h"

#include <errno.h>
#include <math.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

/** Seconds a world without a controller settles before sampling. */
#define WARMUP_S 6

/** Frames the controller must report before sampling starts. */
#define WARMUP_FRAMES 24

/** Seconds to wait for a world to load before giving up on it. */
#define LOAD_TIMEOUT_S 240

/** Default seconds the CPU sample spans; FDT_CPU_SAMPLE_S overrides it. */
#define SAMPLE_S 10

/** Runs per world, reduced by median; FDT_CPU_REPEATS overrides it. */
#define REPEATS 3

/** Frames per second of the DT tick. */
#define DT_HZ 8.0

static int g_sample_s = SAMPLE_S;
static int g_repeats  = REPEATS;
static int g_core     = -1;

/** One world in the sweep. */
typedef struct {
    const char *file;
    int         vessels;
} world_t;

/**
 * The committed worlds, then the generated ones (`make webots-worlds`).
 */
static const world_t WORLDS[] = {
    { "jundia_empty.wbt",  0 },
    { "jundia_single.wbt", 1 },
    { "jundia_fleet.wbt",  2 },
    { "jundia_n05.wbt",    5 },
    { "jundia_n10.wbt",   10 },
    { "jundia_n20.wbt",   20 },
    { "jundia_n30.wbt",   30 },
    { "jundia_n40.wbt",   40 },
    { "jundia_n50.wbt",   50 },
    { "jundia_n60.wbt",   60 },
};
#define NWORLDS ((int)(sizeof WORLDS / sizeof WORLDS[0]))

/** The controller's CPU report, as fdt_controller writes it. */
typedef struct {
    double frames;
    double vessels;
    double injector_cpu_s;
    double controller_cpu_s;
    double worst_update_us;
    int    feasible;
    double wall_s;
    int    valid;
} ctl_report_t;

/** One run of one world. */
typedef struct {
    double total_pct;    /**< WeBots tree, injector included. */
    double injector_pct; /**< The synthetic boats' share. */
    double sim_pct;      /**< total minus injector. */
    double fps;          /**< DT frames per wall second. */
    double worst_us;     /**< Worst update of all twins in one frame. */
} run_t;

/** Clock ticks per second, for interpreting /proc/<pid>/stat. */
static double g_hz;

/** Reads a positive integer from the environment, or keeps the default. */
static int env_int(const char *name, int fallback)
{
    const char *v = getenv(name);
    if (v == NULL || v[0] == '\0') {
        return fallback;
    }
    return atoi(v);
}

/** Wall-clock seconds, monotonic. */
static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

/**
 * @brief CPU time a process has consumed, in seconds.
 *
 * @note Fields 14 and 15 are read by skipping past the comm field, which is
 *       parenthesised and may itself contain spaces and parentheses. Anything
 *       that scans forward from the start instead of from the last ')' gets
 *       this wrong for a process whose name has a space in it.
 */
static double cpu_seconds(pid_t pid)
{
    char path[64];
    snprintf(path, sizeof path, "/proc/%d/stat", (int)pid);

    FILE *f = fopen(path, "r");
    if (f == NULL) {
        return -1.0;
    }

    static char line[4096];
    if (fgets(line, sizeof line, f) == NULL) {
        fclose(f);
        return -1.0;
    }
    fclose(f);

    const char *after_comm = strrchr(line, ')');
    if (after_comm == NULL) {
        return -1.0;
    }

    unsigned long utime = 0;
    unsigned long stime = 0;
    if (sscanf(after_comm + 2,
               "%*s %*s %*s %*s %*s %*s %*s %*s %*s %*s %*s %lu %lu",
               &utime, &stime) != 2) {
        return -1.0;
    }
    return (double)(utime + stime) / g_hz;
}

/**
 * @brief CPU consumed by a process and every descendant it still has.
 */
static double tree_cpu_seconds(pid_t root)
{
    double total = cpu_seconds(root);
    if (total < 0.0) {
        return -1.0;
    }

    char path[64];
    snprintf(path, sizeof path, "/proc/%d/task/%d/children", (int)root,
             (int)root);

    FILE *f = fopen(path, "r");
    if (f == NULL) {
        return total;
    }

    int child = 0;
    while (fscanf(f, "%d", &child) == 1) {
        const double c = tree_cpu_seconds((pid_t)child);
        if (c > 0.0) {
            total += c;
        }
    }
    fclose(f);
    return total;
}

/** Sleeps for a fraction of a second, resuming through signals. */
static void sleep_for(double seconds)
{
    struct timespec req = { .tv_sec = (time_t)seconds,
                            .tv_nsec = (long)((seconds - floor(seconds)) *
                                              1e9) };
    struct timespec rem;
    while (nanosleep(&req, &rem) == -1 && errno == EINTR) {
        req = rem;
    }
}

/** Reads the controller's report; valid is 0 when it is absent. */
static ctl_report_t read_report(const char *path)
{
    ctl_report_t r = {0};
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        return r;
    }
    r.valid = (fscanf(f, "frames %lf vessels %lf injector_cpu_s %lf "
                         "controller_cpu_s %lf worst_update_us %lf "
                         "feasible %d wall_s %lf",
                      &r.frames, &r.vessels, &r.injector_cpu_s,
                      &r.controller_cpu_s, &r.worst_update_us,
                      &r.feasible, &r.wall_s) == 7);
    fclose(f);
    return r;
}

/** Stops the WeBots process group and reaps it. */
static void stop(pid_t pid)
{
    kill(-pid, SIGTERM);
    sleep_for(0.5);
    kill(-pid, SIGKILL);
    waitpid(pid, NULL, 0);
}

/**
 * @brief Runs one world once.
 * @return 0 on success, -1 when WeBots failed to load or stay up.
 */
static int measure(const char *webots, const world_t *w, run_t *out)
{
    char path[512];
    snprintf(path, sizeof path, "adapters/webots/worlds/%s", w->file);

    char rep[128];
    snprintf(rep, sizeof rep, "/tmp/fdt_webots_cpu.%d", (int)getpid());
    (void)remove(rep);

    /* The child redirects stdout, and freopen flushes what the parent had
     * buffered: without this, every earlier line of the report prints twice. */
    fflush(NULL);
    const pid_t pid = fork();
    if (pid < 0) {
        return -1;
    }

    if (pid == 0) {
        setpgid(0, 0);
        if (g_core >= 0) {
            cpu_set_t set;
            CPU_ZERO(&set);
            CPU_SET(g_core, &set);
            (void)sched_setaffinity(0, sizeof set, &set);
        }
        setenv("FDT_CPU_REPORT", rep, 1);
        if (freopen("/dev/null", "w", stdout) == NULL ||
            freopen("/dev/null", "w", stderr) == NULL) {
            _exit(126);
        }
        execl(webots, "webots", "--batch", "--mode=realtime",
              "--no-rendering", "--minimize", path, (char *)NULL);
        _exit(127);
    }

    /* Warmup: until the controller has stepped, or a fixed wait without one. */
    ctl_report_t r0 = {0};
    if (w->vessels == 0) {
        sleep_for(WARMUP_S);
    } else {
        const double give_up = now_s() + LOAD_TIMEOUT_S;
        for (;;) {
            r0 = read_report(rep);
            if (r0.valid && r0.frames >= WARMUP_FRAMES) {
                break;
            }
            if (now_s() > give_up || waitpid(pid, NULL, WNOHANG) == pid) {
                stop(pid);
                return -1;
            }
            sleep_for(0.25);
        }
    }

    r0 = read_report(rep);
    const double c0 = tree_cpu_seconds(pid);
    const double t0 = now_s();
    if (c0 < 0.0) {
        stop(pid);
        return -1;
    }

    sleep_for((double)g_sample_s);

    const ctl_report_t r1 = read_report(rep);
    const double c1 = tree_cpu_seconds(pid);
    const double t1 = now_s();
    stop(pid);
    (void)remove(rep);

    if (c1 < 0.0 || t1 <= t0) {
        return -1;
    }
    const double span = t1 - t0;
    out->total_pct    = 100.0 * (c1 - c0) / span;
    /* The injector share is per wall second of the controller's own
     * reports, which bracket a slightly different span than c0..c1. */
    out->injector_pct = (r0.valid && r1.valid && r1.wall_s > r0.wall_s)
        ? 100.0 * (r1.injector_cpu_s - r0.injector_cpu_s) /
              (r1.wall_s - r0.wall_s)
        : 0.0;
    out->sim_pct      = out->total_pct - out->injector_pct;
    out->fps          = (r0.valid && r1.valid && r1.wall_s > r0.wall_s)
        ? (r1.frames - r0.frames) / (r1.wall_s - r0.wall_s) : 0.0;
    out->worst_us     = r1.valid ? r1.worst_update_us : 0.0;
    return 0;
}

/** Median of @p n values; sorts them in place. */
static double median(double *v, int n)
{
    for (int a = 1; a < n; a++) {
        const double x = v[a];
        int b = a - 1;
        while (b >= 0 && v[b] > x) {
            v[b + 1] = v[b];
            b--;
        }
        v[b + 1] = x;
    }
    return v[n / 2];
}

/** Whether a world file is present. */
static int world_exists(const world_t *w)
{
    char path[512];
    snprintf(path, sizeof path, "adapters/webots/worlds/%s", w->file);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        return 0;
    }
    fclose(f);
    return 1;
}

int main(void)
{
    g_sample_s = env_int("FDT_CPU_SAMPLE_S", SAMPLE_S);
    g_repeats  = env_int("FDT_CPU_REPEATS", REPEATS);
    if (g_sample_s < 1) {
        g_sample_s = SAMPLE_S;
    }
    if (g_repeats < 1) {
        g_repeats = REPEATS;
    }
    if (g_repeats > 64) {
        g_repeats = 64;
    }

    const long ncpu = sysconf(_SC_NPROCESSORS_ONLN);
    g_core = env_int("FDT_CPU_CORE", (ncpu > 1) ? (int)ncpu - 1 : -1);
    if (g_core >= ncpu) {
        g_core = -1;
    }

    g_hz = (double)sysconf(_SC_CLK_TCK);
    if (g_hz <= 0.0) {
        g_hz = 100.0;
    }

    const char *home = getenv("WEBOTS_HOME");
    if (home == NULL || home[0] == '\0') {
        printf("== fleet-dt WeBots CPU bench ==\n");
        printf("WEBOTS_HOME is not set; skipping.\n");
        printf("This benchmark measures C22, which needs the simulator. "
               "See adapters/webots/README.md.\n\n");
        return EXIT_SUCCESS;
    }

    char webots[512];
    snprintf(webots, sizeof webots, "%s/webots", home);

    bench_report_t r;
    bench_open(&r, "webots_cpu");
    bench_banner(&r, "fleet-dt WeBots CPU bench");
    bench_say(&r, "WeBots %s --batch --mode=realtime --no-rendering\n", home);
    if (g_core >= 0) {
        bench_say(&r, "pinned to core %d of %ld (FDT_CPU_CORE); CPU is "
                      "%% of that one core\n", g_core, ncpu);
    } else {
        bench_say(&r, "not pinned (FDT_CPU_CORE=-1); CPU is %% of one "
                      "core and may exceed 100\n");
    }
    bench_say(&r, "sample %d s x %d runs per world, median; "
                  "FDT_CPU_SAMPLE_S and FDT_CPU_REPEATS raise them\n",
              g_sample_s, g_repeats);
    bench_say(&r, "the synthetic boats run in the controller; their share "
                  "is timed\napart and subtracted, so 'sim' is WeBots plus "
                  "the twins.\n");

    static double xs[NWORLDS];
    static double sim[NWORLDS];
    static double inj[NWORLDS];
    static double worst[NWORLDS];
    static double spread[NWORLDS];
    int have[NWORLDS] = {0};

    bench_section(&r, "cost per world");
    bench_say(&r, "  %-18s %4s %9s %9s %9s %7s %6s %11s\n",
              "world", "N", "total", "boats", "sim", "spread", "fps",
              "worst upd.");

    for (int i = 0; i < NWORLDS; i++) {
        const world_t *w = &WORLDS[i];
        if (!world_exists(w)) {
            bench_say(&r, "  %-18s %4d   absent; run `make webots-worlds`\n",
                      w->file, w->vessels);
            continue;
        }

        double tot[64], boats[64], s[64], fps[64];
        double worst_us = 0.0;
        int got = 0;
        for (int rep = 0; rep < g_repeats; rep++) {
            run_t run;
            if (measure(webots, w, &run) == 0) {
                tot[got]   = run.total_pct;
                boats[got] = run.injector_pct;
                s[got]     = run.sim_pct;
                fps[got]   = run.fps;
                if (run.worst_us > worst_us) {
                    worst_us = run.worst_us;
                }
                got++;
            }
        }
        if (got == 0) {
            bench_say(&r, "  %-18s %4d   FAILED to run\n", w->file,
                      w->vessels);
            continue;
        }

        double lo = s[0];
        double hi = s[0];
        for (int k = 1; k < got; k++) {
            lo = (s[k] < lo) ? s[k] : lo;
            hi = (s[k] > hi) ? s[k] : hi;
        }

        have[i]   = 1;
        xs[i]     = (double)w->vessels;
        sim[i]    = median(s, got);
        inj[i]    = median(boats, got);
        worst[i]  = worst_us;
        spread[i] = hi - lo;
        const double total = median(tot, got);
        const double rate  = median(fps, got);

        if (w->vessels == 0) {
            bench_say(&r, "  %-18s %4d %7.2f %% %9s %7.2f %% %5.2f %% %6s "
                          "%11s\n", w->file, 0, total, "-", sim[i],
                      spread[i], "-", "-");
        } else {
            bench_say(&r, "  %-18s %4d %7.2f %% %7.2f %% %7.2f %% %5.2f %% "
                          "%6.2f %8.1f us\n", w->file, w->vessels, total,
                      inj[i], sim[i], spread[i], rate, worst[i]);
        }
    }

    /* Least squares of sim against N, over the worlds with a vessel. */
    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    int m = 0;
    double noise = 0.0;
    for (int i = 0; i < NWORLDS; i++) {
        if (!have[i]) {
            continue;
        }
        noise = (spread[i] > noise) ? spread[i] : noise;
        if (xs[i] < 1.0) {
            continue;
        }
        sx += xs[i];
        sy += sim[i];
        sxx += xs[i] * xs[i];
        sxy += xs[i] * sim[i];
        m++;
    }
    const double den = (double)m * sxx - sx * sx;
    if (m < 2 || den <= 0.0) {
        bench_say(&r, "\nFewer than two worlds with vessels ran; no slope "
                      "can be fitted.\n");
        bench_close(&r);
        return EXIT_SUCCESS;
    }
    const double slope = ((double)m * sxy - sx * sy) / den;
    const double icept = (sy - slope * sx) / (double)m;

    double worst_all = 0.0;
    int at60 = -1;
    for (int i = 0; i < NWORLDS; i++) {
        if (have[i] && worst[i] > worst_all) {
            worst_all = worst[i];
        }
        if (have[i] && WORLDS[i].vessels == 60) {
            at60 = i;
        }
    }

    bench_section(&r, "fit over the worlds with at least one vessel");
    bench_say(&r, "  sim(N) = %.3f %% + %.4f %% x N   (%d worlds)\n",
              icept, slope, m);
    bench_say(&r, "  largest run-to-run spread on this host: %.2f %%\n",
              noise);

    char buf[64];

    bench_section(&r, "against slide 14 (EPYC 7453, one core, rendering off)");
    if (have[0]) {
        snprintf(buf, sizeof buf, "%.2f %%", sim[0]);
        bench_compare_slide(&r, "empty world", buf, "slide 14", "0.9 %",
                            BENCH_HOST);
    }
    snprintf(buf, sizeof buf, "%.3f %%", slope);
    bench_compare_slide(&r, "each added twin", buf, "slide 14", "0.12 %",
                        BENCH_HOST);
    snprintf(buf, sizeof buf, "%.2f %% (fit)", icept + 25.0 * slope);
    bench_compare_slide(&r, "25 boats", buf, "slide 14", "5.0 %",
                        BENCH_HOST);
    if (at60 >= 0) {
        snprintf(buf, sizeof buf, "%.2f %%", sim[at60]);
        bench_compare_slide(&r, "60 boats", buf, "slide 14", "8.3 %",
                            BENCH_HOST);
    }
    snprintf(buf, sizeof buf, "%.1f us", worst_all);
    bench_compare_slide(&r, "worst update, all twins", buf, "slide 14",
                        "36 us", BENCH_HOST);
    snprintf(buf, sizeof buf, "%.0fx", (worst_all > 0.0)
             ? 1e6 / DT_HZ / worst_all : 0.0);
    bench_compare_slide(&r, "slack to 125 ms", buf, "slide 14", "3500x",
                        BENCH_HOST);

    bench_section(&r, "against Section V-A (i7-6700HQ, rendering on)");
    if (have[0] && have[1]) {
        snprintf(buf, sizeof buf, "%.2f %%", sim[1] - sim[0]);
        bench_compare(&r, "first boat", buf, "adds 10 % CPU", BENCH_HOST);
    }
    snprintf(buf, sizeof buf, "%.3f %% (slope)", slope);
    bench_compare(&r, "each subsequent boat", buf, "less than 1 %",
                  (slope < 1.0) ? BENCH_MATCH : BENCH_HOST);
    bench_say(&r,
        "\n  The paper ran with the 3D view on and states an inequality for\n"
        "  each boat after the first; the slope above is the like quantity\n"
        "  here. Slide 14 ran rendering off on another machine, so its\n"
        "  figures are printed for scale and never marked as matched.\n");

    /* Charts: CPU against N with the fit, and the worst update. The fit is
     * evaluated at each measured N, because the CSV writer pairs series by
     * row against the first series' x. */
    static double px[NWORLDS], py[NWORLDS], fy[NWORLDS];
    static double wx[NWORLDS], wy[NWORLDS];
    int np = 0, nw = 0;
    for (int i = 0; i < NWORLDS; i++) {
        if (!have[i]) {
            continue;
        }
        px[np] = xs[i];
        py[np] = sim[i];
        fy[np] = icept + slope * xs[i];
        np++;
        if (WORLDS[i].vessels > 0) {
            wx[nw] = xs[i];
            wy[nw] = worst[i];
            nw++;
        }
    }

    fdt_plot_t p;
    fdt_plot_init(&p, "CPU of the simulation against boats in the world",
                  "boats in the simulator, N", "CPU (% of one core)");
    fdt_plot_series(&p, "measured (sim)", px, py, (size_t)np,
                    FDT_PLOT_SCATTER);
    fdt_plot_series(&p, "least-squares fit", px, fy, (size_t)np,
                    FDT_PLOT_LINE);
    fdt_plot_hline(&p, 8.3, "slide 14: 8.3 % at 60");
    fdt_plot_hline(&p, 0.9, "slide 14: empty world 0.9 %");
    fdt_plot_write_svg(&p, "results/webots_cpu.svg");
    fdt_plot_write_csv(&p, "results/webots_cpu.csv");

    fdt_plot_t u;
    fdt_plot_init(&u, "Worst update of all twins in one frame",
                  "boats in the simulator, N", "time (us)");
    fdt_plot_series(&u, "worst update", wx, wy, (size_t)nw, FDT_PLOT_LINE);
    fdt_plot_hline(&u, 36.0, "slide 14: 36 us");
    fdt_plot_write_svg(&u, "results/webots_update.svg");
    fdt_plot_write_csv(&u, "results/webots_update.csv");

    bench_artefacts(&r, "webots_cpu");
    bench_say(&r, "           results/webots_update.csv, "
                  "results/webots_update.svg\n");
    bench_close(&r);
    return EXIT_SUCCESS;
}
