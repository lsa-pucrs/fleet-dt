/*
 * fdt_gsstamp.so: Mosquitto 2.1 plugin (plugin API version 5) of the GS MQTT broker that measures the
 * delivery time of the broker, delta_GS of the paper (Section VI-E; supplementary Section S6: "the GS MQTT
 * broker timestamps the reception and the delivery of each message"). The logic lives in fdt_gsstamp_core.c.
 *
 *   MOSQ_EVT_MESSAGE_IN   reception of a PUBLISH, after the broker has read the whole packet from the data
 *                         bridge (src/handle_publish.c, before the routing to the subscribers)
 *   MOSQ_EVT_MESSAGE_OUT  delivery: the broker writes the PUBLISH to a subscriber (lib/send_publish.c,
 *                         before packet__queue, which writes it to the socket)
 *
 * Both events take CLOCK_MONOTONIC. The plugin matches the uplink messages of the data bridges (topics
 * boat/b<i>/tx/<name>) by topic and step header and writes one JSON line per step t to plugin_opt_log
 * (fdt_gsstamp_core.h). The messages of other topics continue untouched; the plugin never changes or
 * refuses a message.
 *
 * mosquitto.conf of the GS (fleetdt.gsconf gs --stamp LOG writes these lines):
 *   plugin /opt/fleet-dt/lib/fdt_gsstamp.so
 *   plugin_opt_log /tmp/fdt-run/gsdelivery.jsonl
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <mosquitto.h>

#include "fdt_gsstamp_core.h"

MOSQUITTO_PLUGIN_DECLARE_VERSION(5);

static struct fdt_gss *G;
static mosquitto_plugin_id_t *PID;

static int64_t now_ns(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static int on_in(int event, void *event_data, void *userdata)
{
	(void)event;
	(void)userdata;
	struct mosquitto_evt_message *m = event_data;
	if (G && m)
		fdt_gss_in(G, m->topic, m->payload, m->payloadlen, now_ns());
	return MOSQ_ERR_SUCCESS;
}

static int on_out(int event, void *event_data, void *userdata)
{
	(void)event;
	(void)userdata;
	struct mosquitto_evt_message *m = event_data;
	if (G && m)
		fdt_gss_out(G, m->topic, m->payload, m->payloadlen, now_ns());
	return MOSQ_ERR_SUCCESS;
}

int mosquitto_plugin_init(mosquitto_plugin_id_t *identifier, void **userdata, struct mosquitto_opt *opts,
                          int opt_count)
{
	(void)userdata;
	const char *path = NULL;
	for (int i = 0; i < opt_count; i++)
		if (!strcmp(opts[i].key, "log"))
			path = opts[i].value;
	if (!path) {
		mosquitto_log_printf(MOSQ_LOG_ERR, "fdt_gsstamp: plugin_opt_log is required");
		return MOSQ_ERR_INVAL;
	}
	FILE *f = fopen(path, "a");
	if (!f) {
		mosquitto_log_printf(MOSQ_LOG_ERR, "fdt_gsstamp: cannot open %s", path);
		return MOSQ_ERR_INVAL;
	}
	G = malloc(sizeof *G);
	if (!G) {
		fclose(f);
		return MOSQ_ERR_NOMEM;
	}
	fdt_gss_init(G, f);
	PID = identifier;
	if (mosquitto_callback_register(identifier, MOSQ_EVT_MESSAGE_IN, on_in, NULL, NULL) ||
	    mosquitto_callback_register(identifier, MOSQ_EVT_MESSAGE_OUT, on_out, NULL, NULL)) {
		fclose(f);
		free(G);
		G = NULL;
		return MOSQ_ERR_UNKNOWN;
	}
	mosquitto_plugin_set_info(identifier, "fdt_gsstamp", "1");
	mosquitto_log_printf(MOSQ_LOG_NOTICE, "fdt_gsstamp: delivery times of boat/+/tx/# to %s", path);
	return MOSQ_ERR_SUCCESS;
}

int mosquitto_plugin_cleanup(void *userdata, struct mosquitto_opt *opts, int opt_count)
{
	(void)userdata;
	(void)opts;
	(void)opt_count;
	if (!G)
		return MOSQ_ERR_SUCCESS;
	fdt_gss_flush_all(G);
	mosquitto_log_printf(MOSQ_LOG_NOTICE,
	                     "fdt_gsstamp: %llu receptions, %llu deliveries, %llu unmatched, %llu outside the ring, "
	                     "largest delivery time %.1f us (step %lld)",
	                     (unsigned long long)G->n_in, (unsigned long long)G->n_out,
	                     (unsigned long long)G->unmatched, (unsigned long long)G->lost, (double)G->max_ns / 1e3,
	                     (long long)G->max_t);
	if (G->log)
		fclose(G->log);
	free(G);
	G = NULL;
	return MOSQ_ERR_SUCCESS;
}
