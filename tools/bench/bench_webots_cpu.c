/**
 * @file bench_webots_cpu.c
 * @brief The CPU cost of adding a vessel to the simulation, Section V-A.
 */
#define _POSIX_C_SOURCE 200809L

#include "bench_common.h"

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

/** Seconds of settling before sampling starts, so startup is not counted. */
#define WARMUP_S 6

/** Default seconds the CPU sample spans; FDT_CPU_SAMPLE_S overrides it. */
#define SAMPLE_S 10

/**
 * Runs per world, reduced by median.
 */
#define REPEATS 3

/** Resolved from the environment at startup, so a careful run costs no rebuild. */
static int g_sample_s = SAMPLE_S;
static int g_repeats  = REPEATS;

/** Reads a positive integer from the environment, or keeps the default. */
static int env_int(const char *name, int fallback)
{
    const char *v = getenv(name);
    if (v == NULL || v[0] == '\0') {
        return fallback;
    }
    const int n = atoi(v);
    return (n > 0) ? n : fallback;
}

/** One world in the sweep. */
typedef struct {
    const char *file;
    int         vessels;
    const char *what;
} world_t;

static const world_t WORLDS[] = {
    { "jundia_empty.wbt",  0, "renderer only" },
    { "jundia_single.wbt", 1, "first boat" },
    { "jundia_fleet.wbt",  2, "second boat" },
};
#define NWORLDS ((int)(sizeof WORLDS / sizeof WORLDS[0]))

/** Clock ticks per second, for interpreting /proc/<pid>/stat. */
static double g_hz;

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

    /* One level of children is enough: WeBots spawns controllers directly. */
    char path[64];
    snprintf(path, sizeof path, "/proc/%d/task/%d/children", (int)root,
             (int)root);

    FILE *f = fopen(path, "r");
    if (f == NULL) {
        return total;
    }

    int child = 0;
    while (fscanf(f, "%d", &child) == 1) {
        const double c = cpu_seconds((pid_t)child);
        if (c > 0.0) {
            total += c;
        }
    }
    fclose(f);
    return total;
}

/** Sleeps for a whole number of seconds, resuming through signals. */
static void sleep_s(int seconds)
{
    struct timespec req = { .tv_sec = seconds, .tv_nsec = 0 };
    struct timespec rem;
    while (nanosleep(&req, &rem) == -1 && errno == EINTR) {
        req = rem;
    }
}

/**
 * @brief Runs one world and returns the CPU percentage it consumed.
 * @return Percent of one core, or -1.0 when WeBots failed to stay up.
 */
static double measure(const char *webots, const char *world)
{
    char path[512];
    snprintf(path, sizeof path, "adapters/webots/worlds/%s", world);

    const pid_t pid = fork();
    if (pid < 0) {
        return -1.0;
    }

    if (pid == 0) {
        setpgid(0, 0);
        freopen("/dev/null", "w", stdout);
        freopen("/dev/null", "w", stderr);
        execl(webots, "webots", "--batch", "--mode=realtime", "--minimize",
              path, (char *)NULL);
        _exit(127);
    }

    sleep_s(WARMUP_S);

    const double c0 = tree_cpu_seconds(pid);
    const double t0 = now_s();
    if (c0 < 0.0) {
        kill(-pid, SIGKILL);
        waitpid(pid, NULL, 0);
        return -1.0;
    }

    sleep_s(g_sample_s);

    const double c1 = tree_cpu_seconds(pid);
    const double t1 = now_s();

    kill(-pid, SIGTERM);
    kill(-pid, SIGKILL);
    waitpid(pid, NULL, 0);

    if (c1 < 0.0 || t1 <= t0) {
        return -1.0;
    }
    return 100.0 * (c1 - c0) / (t1 - t0);
}

int main(void)
{
    g_sample_s = env_int("FDT_CPU_SAMPLE_S", SAMPLE_S);
    g_repeats  = env_int("FDT_CPU_REPEATS", REPEATS);
    if (g_repeats > 64) {
        g_repeats = 64;
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
    bench_say(&r, "warmup %d s, sample %d s x %d runs per world, "
                  "%.0f clock ticks/s\n",
              WARMUP_S, g_sample_s, g_repeats, g_hz);
    bench_say(&r, "raise FDT_CPU_SAMPLE_S and FDT_CPU_REPEATS for a careful "
                  "run on a quiet host.\n");
    bench_say(&r, "measuring WeBots and its controller together: adding a boat "
                  "adds\nboth a hull to render and a twin to step.\n");

    static double xs[NWORLDS];
    static double cpu[NWORLDS];
    static double spread[NWORLDS];
    static double increment[NWORLDS];

    bench_section(&r, "absolute cost per world");
    bench_say(&r, "  median of %d runs each; spread is max minus min\n",
              g_repeats);
    bench_say(&r, "  %-20s %8s %11s %11s %12s\n",
              "world", "vessels", "cpu median", "spread", "increment");

    int ok = 1;
    for (int i = 0; i < NWORLDS; i++) {
        double runs[64];
        int got = 0;

        for (int rep = 0; rep < g_repeats; rep++) {
            const double pct = measure(webots, WORLDS[i].file);
            if (pct >= 0.0) {
                runs[got++] = pct;
            }
        }
        if (got == 0) {
            bench_say(&r, "  %-20s   FAILED to run\n", WORLDS[i].file);
            ok = 0;
            continue;
        }

        /* Median over a handful of runs: an insertion sort is the whole of it. */
        for (int a = 1; a < got; a++) {
            const double v = runs[a];
            int b = a - 1;
            while (b >= 0 && runs[b] > v) {
                runs[b + 1] = runs[b];
                b--;
            }
            runs[b + 1] = v;
        }

        xs[i]     = (double)WORLDS[i].vessels;
        cpu[i]    = runs[got / 2];
        spread[i] = runs[got - 1] - runs[0];
        increment[i] = (i == 0) ? cpu[i] : (cpu[i] - cpu[i - 1]);

        bench_say(&r, "  %-20s %8d %9.2f %% %9.2f %% %10.2f %%   (%s)\n",
                  WORLDS[i].file, WORLDS[i].vessels, cpu[i], spread[i],
                  increment[i], WORLDS[i].what);
    }

    if (!ok) {
        bench_say(&r, "\nAt least one world failed to run; the differences "
                      "below are not meaningful.\n");
        bench_close(&r);
        return EXIT_SUCCESS;
    }

    char buf[64];

    const double noise = (spread[0] > spread[1]) ? spread[0] : spread[1];
    const double noise2 = (noise > spread[2]) ? noise : spread[2];
    (void)noise;

    bench_section(&r, "the two figures Section V-A publishes");

    snprintf(buf, sizeof buf, "%.2f %%", increment[1]);
    bench_compare(&r, "first boat", buf, "adds 10 % CPU",
                  (increment[1] > 5.0 && increment[1] < 20.0)
                      ? BENCH_MATCH : BENCH_HOST);

    snprintf(buf, sizeof buf, "%.2f %%", increment[2]);
    bench_compare(&r, "each subsequent boat", buf, "less than 1 %",
                  (increment[2] < 1.0 && increment[2] >= noise2)
                      ? BENCH_MATCH : BENCH_HOST);

    bench_say(&r, "\n  run-to-run spread on this host: %.2f %%. Increments "
                  "closer together\n  than the spread carry it, so raise "
                  "FDT_CPU_SAMPLE_S and\n  FDT_CPU_REPEATS for a finer "
                  "figure.\n", noise2);

    bench_say(&r,
        "\n  Section V-A states that the first vessel costs an order more\n"
        "  than each vessel after it, because the renderer, the physics\n"
        "  world and the fluid run once regardless of vessel count. That\n"
        "  ratio is what makes a fleet tractable on one host.\n");

    if (increment[2] > 0.0 && increment[2] >= noise2) {
        bench_say(&r, "\n  ratio first : subsequent = %.1f : 1\n",
                  increment[1] / increment[2]);
    } else {
        bench_say(&r,
            "\n  No ratio is printed here: the subsequent-boat increment is\n"
            "  smaller than this host's run-to-run spread, so the quotient\n"
            "  would carry the spread rather than the measurement. Longer\n"
            "  sampling is the lever: raise FDT_CPU_SAMPLE_S and\n"
            "  FDT_CPU_REPEATS on a quieter host.\n");
    }

    fdt_plot_t p;
    fdt_plot_init(&p, "CPU cost of the simulation against vessel count",
                  "vessels in the world", "CPU (% of one core)");
    fdt_plot_series(&p, "total", xs, cpu, NWORLDS, FDT_PLOT_LINE);
    fdt_plot_series(&p, "increment per vessel", xs, increment, NWORLDS,
                    FDT_PLOT_BAR);
    fdt_plot_hline(&p, 10.0, "first boat: 10 % (Sec. V-A)");
    fdt_plot_hline(&p, 1.0, "each next: < 1 % (Sec. V-A)");
    fdt_plot_write_svg(&p, "results/webots_cpu.svg");
    fdt_plot_write_csv(&p, "results/webots_cpu.csv");

    bench_artefacts(&r, "webots_cpu");
    bench_close(&r);
    return EXIT_SUCCESS;
}
