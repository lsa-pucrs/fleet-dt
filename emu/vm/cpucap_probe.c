/* cpucap_probe: busy loop that records its own stalls (gaps > --gap-us between two clock readings), to
 * measure what a CPU cap does to a CPU-bound program: run bursts and stalls (us), one line per stall:
 * "run_us stall_us".   cpucap_probe --seconds S [--gap-us 30] > trace */
#define _GNU_SOURCE
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static int64_t mono(void) {
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

int main(int argc, char **argv) {
	double secs = 10, gap_us = 30;
	for (int i = 1; i + 1 < argc; i++) {
		if (!strcmp(argv[i], "--seconds")) secs = atof(argv[++i]);
		else if (!strcmp(argv[i], "--gap-us")) gap_us = atof(argv[++i]);
	}
	int64_t t0 = mono(), last = t0, run0 = t0, end = t0 + (int64_t)(secs * 1e9), G = (int64_t)(gap_us * 1e3);
	while (last < end) {
		int64_t t = mono();
		if (t - last > G) {
			printf("%.1f %.1f\n", (last - run0) / 1e3, (t - last) / 1e3);
			run0 = t;
		}
		last = t;
	}
	return 0;
}
