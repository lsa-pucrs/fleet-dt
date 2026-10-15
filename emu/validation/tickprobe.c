/*
 * Tick probe: a Mosquitto 2.1 plugin (API 5) that asks for a MOSQ_EVT_TICK every plugin_opt_period_us
 * (default 1000) and measures the lateness of each tick against its deadline (CLOCK_MONOTONIC), the
 * check of the patch 0001 (keep the pending deadline when another event wakes the loop) and of the 1 ms
 * ticks that the vessel regulator needs (PR1). Writes a histogram to plugin_opt_out at cleanup:
 *   {"ticks": n, "late_us": {"p50", "p90", "p99", "p999", "max"}, "over_1ms": k, "early": e}
 *
 *   cc -O2 -fPIC -shared -I/opt/mosquitto-2.1/include -o tickprobe.so emu/validation/tickprobe.c
 *   mosquitto.conf: plugin /path/tickprobe.so, plugin_opt_out /tmp/tick.json, plugin_opt_period_us 1000
 */
#define _GNU_SOURCE
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <mosquitto.h>

MOSQUITTO_PLUGIN_DECLARE_VERSION(5);

#define NBIN 100001   /* 1 us bins up to 100 ms */

static mosquitto_plugin_id_t *pid;
static uint32_t *hist;
static uint64_t ticks, early, over;
static int64_t deadline, period_us = 1000, maxlate;
static char out[256] = "/tmp/tickprobe.json";

static int64_t mono_us(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

static int on_tick(int event, void *event_data, void *userdata)
{
	struct mosquitto_evt_tick *t = event_data;
	(void)event;
	(void)userdata;
	int64_t now = mono_us();
	if (deadline) {
		int64_t late = now - deadline;
		if (late < 0) {
			early++;
		} else {
			ticks++;
			if (late > maxlate)
				maxlate = late;
			if (late > 1000)
				over++;
			hist[late < NBIN ? late : NBIN - 1]++;
		}
		if (late < 0) {   /* woken before the deadline: keep it */
			int64_t rest = deadline - now;
			long ms = (long)((rest + 999) / 1000);
			t->next_s = ms / 1000;
			t->next_ms = ms % 1000;
			return MOSQ_ERR_SUCCESS;
		}
	}
	deadline = now + period_us;
	long ms = (long)((period_us + 999) / 1000);
	t->next_s = ms / 1000;
	t->next_ms = ms % 1000;
	return MOSQ_ERR_SUCCESS;
}

static int64_t quant(double q)
{
	uint64_t want = (uint64_t)(q * (double)ticks), acc = 0;
	for (int k = 0; k < NBIN; k++) {
		acc += hist[k];
		if (acc > want)
			return k;
	}
	return NBIN - 1;
}

int mosquitto_plugin_init(mosquitto_plugin_id_t *identifier, void **userdata, struct mosquitto_opt *opts,
                          int opt_count)
{
	(void)userdata;
	pid = identifier;
	for (int i = 0; i < opt_count; i++) {
		if (!strcmp(opts[i].key, "out"))
			snprintf(out, sizeof out, "%s", opts[i].value);
		else if (!strcmp(opts[i].key, "period_us"))
			period_us = atoll(opts[i].value);
	}
	hist = calloc(NBIN, sizeof *hist);
	if (!hist)
		return MOSQ_ERR_NOMEM;
	return mosquitto_callback_register(pid, MOSQ_EVT_TICK, on_tick, NULL, NULL);
}

int mosquitto_plugin_cleanup(void *userdata, struct mosquitto_opt *opts, int opt_count)
{
	(void)userdata;
	(void)opts;
	(void)opt_count;
	FILE *f = fopen(out, "w");
	if (f) {
		fprintf(f, "{\"ticks\":%llu,\"period_us\":%lld,\"late_us\":{\"p50\":%lld,\"p90\":%lld,\"p99\":%lld,"
		           "\"p999\":%lld,\"max\":%lld},\"over_1ms\":%llu,\"early\":%llu}\n",
		        (unsigned long long)ticks, (long long)period_us, (long long)quant(0.5), (long long)quant(0.9),
		        (long long)quant(0.99), (long long)quant(0.999), (long long)maxlate, (unsigned long long)over,
		        (unsigned long long)early);
		fclose(f);
	}
	free(hist);
	return MOSQ_ERR_SUCCESS;
}
