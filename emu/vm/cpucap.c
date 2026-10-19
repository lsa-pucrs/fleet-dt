/* fdt_cpucap: user-space CPU bandwidth cap of processes (the CFS bandwidth rule of cgroup cpu.max, for hosts
 * where the user cannot write cpu.max: cgroup v1 without delegation, no root; emu/vm/cpu_profile.py).
 *
 *   fdt_cpucap --quota-us Q --period-us P [--poll-us 100] [--lead-us POLL/2] [--source perf|clock] [--stats FILE]
 *              [--trace FILE] PID [PID ...]
 * (--lead-us: the stop comes when the CPU time of the period reaches Q - lead, since the detection overruns
 * by about half a poll interval on average)
 *
 * Every poll interval (absolute deadlines on CLOCK_MONOTONIC) it reads the CPU time of each process and,
 * per period P aligned on the start of the program, sends SIGSTOP to a process
 * whose CPU time in the period reached Q and SIGCONT at the next period boundary. Like cpu.max, unused
 * quota does not carry over. A process that the controller stopped is always continued on exit (SIGTERM,
 * SIGINT, SIGHUP) and when its period ends.
 * CPU time (--source): "perf" (default) sums a task-clock counter (perf_event_open, one per thread, threads
 * rescanned every 100 ms; reading a counter of a running thread updates it, so the value is exact at the
 * read; needs perf_event_paranoid <= 2 for processes of the same user); "clock" reads the process CPU clock
 * (clock_getcpuclockid), which the kernel of RHEL 8 updates only at the scheduler tick (1 ms at HZ 1000),
 * so the cap overruns the quota by up to a tick.
 * Statistics per process (FILE, JSON, rewritten every second and on exit): periods, nr_throttled,
 * throttled_usec, used_usec, overrun (CPU time beyond Q in a throttled period: detection and signal
 * delay), resume lateness (SIGCONT after the boundary) and stop lateness (SIGSTOP after the poll that saw
 * the quota), as p50/p99/max in us. --trace writes one line per throttled period (pid, k, used_us, overrun_us,
 * stop_late_us, resume_late_us).
 */
#define _GNU_SOURCE
#include <errno.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <dirent.h>
#include <linux/perf_event.h>
#include <sys/syscall.h>

#define MAXP 64
#define MAXT 256
#define HIST 4096          /* histogram bins of 10 us (0 .. 40.96 ms) */

struct proc {
	pid_t pid;
	clockid_t clk;
	int alive, stopped;
	int64_t base_ns;          /* CPU time at the start of the period */
	int64_t used_total_ns, throttled_ns, stop_at_ns;
	long periods, nr_throttled;
	unsigned hist_over[HIST], hist_resume[HIST], hist_stop[HIST];
	double max_over, max_resume, max_stop;
	int nt, tid[MAXT], fd[MAXT];
};
static int use_perf = 1;

static int has_tid(struct proc *p, int t) {
	for (int i = 0; i < p->nt; i++)
		if (p->tid[i] == t) return 1;
	return 0;
}

static void scan_threads(struct proc *p) {
	char path[64];
	snprintf(path, sizeof path, "/proc/%d/task", p->pid);
	DIR *d = opendir(path);
	if (!d) return;
	struct dirent *e;
	while ((e = readdir(d)) && p->nt < MAXT) {
		int t = atoi(e->d_name);
		if (t <= 0 || has_tid(p, t)) continue;
		struct perf_event_attr a;
		memset(&a, 0, sizeof a);
		a.size = sizeof a;
		a.type = PERF_TYPE_SOFTWARE;
		a.config = PERF_COUNT_SW_TASK_CLOCK;
		a.exclude_kernel = 0;
		int fd = syscall(SYS_perf_event_open, &a, t, -1, -1, 0);
		if (fd < 0) {
			a.exclude_kernel = 1;      /* perf_event_paranoid 2 */
			a.exclude_hv = 1;
			fd = syscall(SYS_perf_event_open, &a, t, -1, -1, 0);
		}
		if (fd < 0) continue;
		p->tid[p->nt] = t;
		p->fd[p->nt++] = fd;
	}
	closedir(d);
}

static struct proc P[MAXP];
static int NP;
static volatile sig_atomic_t quit;
static int64_t T0;

static int64_t mono(void) {
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static int cputime(struct proc *p, int64_t *v) {
	if (use_perf) {
		int64_t sum = 0;
		for (int i = 0; i < p->nt; i++) {
			uint64_t c;
			if (read(p->fd[i], &c, sizeof c) == sizeof c) sum += (int64_t)c;
		}
		*v = sum;
		return kill(p->pid, 0) == 0 || errno == EPERM ? 0 : -1;
	}
	struct timespec ts;
	if (clock_gettime(p->clk, &ts) != 0)
		return -1;
	*v = (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
	return 0;
}

static void on_sig(int s) { (void)s; quit = 1; }

static void hadd(unsigned *h, double *mx, double us) {
	int b = us < 0 ? 0 : (int)(us / 10.0);
	if (b >= HIST) b = HIST - 1;
	h[b]++;
	if (us > *mx) *mx = us;
}

static double hq(const unsigned *h, double q) {
	unsigned long n = 0, c = 0;
	for (int i = 0; i < HIST; i++) n += h[i];
	if (!n) return 0;
	for (int i = 0; i < HIST; i++) {
		c += h[i];
		if (c >= q * n) return (i + 0.5) * 10.0;
	}
	return HIST * 10.0;
}

static void write_stats(const char *path, long quota_us, long period_us, long poll_us) {
	if (!path) return;
	char tmp[4096];
	snprintf(tmp, sizeof tmp, "%s.tmp", path);
	FILE *f = fopen(tmp, "w");
	if (!f) return;
	double el = (mono() - T0) / 1e3;
	fprintf(f, "{\"quota_us\": %ld, \"period_us\": %ld, \"poll_us\": %ld, \"source\": \"%s\", \"elapsed_us\": %.0f, "
		"\"procs\": {", quota_us, period_us, poll_us, use_perf ? "perf" : "clock", el);
	for (int i = 0; i < NP; i++) {
		struct proc *p = &P[i];
		fprintf(f, "%s\"%d\": {\"alive\": %d, \"periods\": %ld, \"nr_throttled\": %ld, \"throttled_usec\": %.0f, "
			"\"used_usec\": %.0f, \"ratio\": %.4f, "
			"\"overrun_us\": {\"p50\": %.0f, \"p99\": %.0f, \"max\": %.1f}, "
			"\"resume_late_us\": {\"p50\": %.0f, \"p99\": %.0f, \"max\": %.1f}, "
			"\"stop_late_us\": {\"p50\": %.0f, \"p99\": %.0f, \"max\": %.1f}}",
			i ? ", " : "", p->pid, p->alive, p->periods, p->nr_throttled, p->throttled_ns / 1e3,
			p->used_total_ns / 1e3, el > 0 ? p->used_total_ns / 1e3 / el : 0.0,
			hq(p->hist_over, .5), hq(p->hist_over, .99), p->max_over,
			hq(p->hist_resume, .5), hq(p->hist_resume, .99), p->max_resume,
			hq(p->hist_stop, .5), hq(p->hist_stop, .99), p->max_stop);
	}
	fprintf(f, "}}\n");
	fclose(f);
	rename(tmp, path);
}

int main(int argc, char **argv) {
	long quota_us = 1000, period_us = 4000, poll_us = 100, lead_us = -1;
	const char *stats = NULL, *trace = NULL;
	int i = 1;
	for (; i < argc && argv[i][0] == '-'; i++) {
		if (!strcmp(argv[i], "--quota-us") && i + 1 < argc) quota_us = atol(argv[++i]);
		else if (!strcmp(argv[i], "--period-us") && i + 1 < argc) period_us = atol(argv[++i]);
		else if (!strcmp(argv[i], "--poll-us") && i + 1 < argc) poll_us = atol(argv[++i]);
		else if (!strcmp(argv[i], "--lead-us") && i + 1 < argc) lead_us = atol(argv[++i]);
		else if (!strcmp(argv[i], "--stats") && i + 1 < argc) stats = argv[++i];
		else if (!strcmp(argv[i], "--trace") && i + 1 < argc) trace = argv[++i];
		else if (!strcmp(argv[i], "--source") && i + 1 < argc) use_perf = !strcmp(argv[++i], "perf");
		else { fprintf(stderr, "unknown option %s\n", argv[i]); return 2; }
	}
	if (quota_us <= 0 || period_us <= quota_us || poll_us <= 0 || i >= argc) {
		fprintf(stderr, "usage: fdt_cpucap --quota-us Q --period-us P [--poll-us U] [--stats F] [--trace F] PID...\n");
		return 2;
	}
	for (; i < argc && NP < MAXP; i++) {
		struct proc *p = &P[NP];
		memset(p, 0, sizeof *p);
		p->pid = atoi(argv[i]);
		if (clock_getcpuclockid(p->pid, &p->clk) != 0) { fprintf(stderr, "no CPU clock of %d\n", p->pid); return 1; }
		p->alive = 1;
		if (use_perf) {
			scan_threads(p);
			if (!p->nt) { fprintf(stderr, "no task-clock counter for %d (perf_event_paranoid?)\n", p->pid); return 1; }
		}
		NP++;
	}
	FILE *tf = trace ? fopen(trace, "w") : NULL;
	struct sigaction sa;
	memset(&sa, 0, sizeof sa);
	sa.sa_handler = on_sig;
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGHUP, &sa, NULL);
	if (lead_us < 0) lead_us = poll_us / 2;   /* stop half a poll early: the mean overrun of the detection */
	const int64_t Q = quota_us * 1000LL, PER = period_us * 1000LL, POLL = poll_us * 1000LL, LEAD = lead_us * 1000LL;
	T0 = mono();
	int64_t pstart = T0, next = T0, last_stats = T0;
	long k = 0;
	for (int j = 0; j < NP; j++) cputime(&P[j], &P[j].base_ns);
	while (!quit) {
		next += POLL;
		struct timespec ts = {next / 1000000000LL, next % 1000000000LL};
		while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) == EINTR && !quit)
			;
		int64_t now = mono();
		if (now >= pstart + PER) {          /* period boundary (or several, after a late wakeup) */
			int64_t nb = pstart + PER * ((now - pstart) / PER);
			double late = (now - (pstart + PER)) / 1e3;
			for (int j = 0; j < NP; j++) {
				struct proc *p = &P[j];
				if (!p->alive) continue;
				int64_t v;
				if (cputime(p, &v) != 0) { p->alive = 0; continue; }
				p->used_total_ns += v - p->base_ns;
				p->periods += (now - pstart) / PER;
				if (p->stopped) {
					kill(p->pid, SIGCONT);
					p->stopped = 0;
					p->throttled_ns += now - p->stop_at_ns;
					hadd(p->hist_resume, &p->max_resume, late);
					if (tf) fprintf(tf, "%d %ld %.1f %.1f\n", p->pid, k, (v - p->base_ns) / 1e3, late);
				}
				p->base_ns = v;
			}
			k += (now - pstart) / PER;
			pstart = nb;
			if (next < now) next = now;
		}
		for (int j = 0; j < NP; j++) {
			struct proc *p = &P[j];
			if (!p->alive || p->stopped) continue;
			int64_t v;
			if (cputime(p, &v) != 0) { p->alive = 0; continue; }
			if (v - p->base_ns >= Q - LEAD) {
				kill(p->pid, SIGSTOP);
				int64_t t1 = mono();
				int64_t v2 = v;
				cputime(p, &v2);           /* the slice that ran until the stop took effect */
				p->stopped = 1;
				p->stop_at_ns = t1;
				p->nr_throttled++;
				hadd(p->hist_over, &p->max_over, (v2 - p->base_ns - Q) / 1e3);
				hadd(p->hist_stop, &p->max_stop, (t1 - now) / 1e3);
			}
		}
		if (use_perf && (k % 25) == 0 && now - pstart < POLL) {   /* new threads, every 25 periods */
			for (int j = 0; j < NP; j++)
				if (P[j].alive) {
					int64_t before, after;
					cputime(&P[j], &before);
					scan_threads(&P[j]);
					cputime(&P[j], &after);
					P[j].base_ns += after - before;   /* a new thread's earlier time is not in this period */
				}
		}
		if (now - last_stats > 1000000000LL) {
			write_stats(stats, quota_us, period_us, poll_us);
			if (tf) fflush(tf);
			last_stats = now;
		}
	}
	for (int j = 0; j < NP; j++)
		if (P[j].stopped) kill(P[j].pid, SIGCONT);
	write_stats(stats, quota_us, period_us, poll_us);
	if (tf) fclose(tf);
	return 0;
}
