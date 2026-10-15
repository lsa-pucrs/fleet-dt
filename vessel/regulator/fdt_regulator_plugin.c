/*
 * Fleet-DT vessel regulator: Mosquitto 2.1 plugin (plugin API version 5) of the
 * vessel MQTT broker (Definition 13, R1). The logic lives in fdt_reg.c.
 *
 *   MOSQ_EVT_MESSAGE_IN  copies each message of boat/b<i>/raw/<topic>, and reads
 *                        boat/b<i>/demand (DTI program) and boat/b<i>/levels (AS,
 *                        through the control bridge); every message continues to
 *                        the local subscribers
 *   MOSQ_EVT_TICK        phase 3 of each step and the publication of the step log
 *                        (needs patch 0001: the broker keeps the tick deadline)
 *   MOSQ_EVT_CONNECT     finds the socket of the data bridge (listener data_port)
 *   MOSQ_EVT_DISCONNECT  forgets it
 *
 * mosquitto.conf:
 *   plugin /opt/fleet-dt/lib/fdt_regulator.so
 *   plugin_opt_dtp /etc/fleetdt/dtp.json      DTP file (required)
 *   plugin_opt_vessel 1                       vessel index i (required)
 *   plugin_opt_data_port 1884                 listener of the data bridge
 *   plugin_opt_max_msgs 1024                  queue capacity per topic and step
 *   plugin_opt_log_topic boat/b1/regulator    step log ("none" disables it)
 *   plugin_opt_stats_s 10                     period of the statistics line in the broker log
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <dirent.h>
#include <errno.h>
#include <linux/sockios.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include <mosquitto.h>

#include "fdt_common.h"
#include "fdt_reg.h"

MOSQUITTO_PLUGIN_DECLARE_VERSION(5);

#define TOPIC_MAX 160

static struct {
	mosquitto_plugin_id_t *pid;
	struct fdt_dtp dtp;
	struct fdt_reg reg;
	int vessel, data_port, max_msgs;
	char raw_prefix[TOPIC_MAX], demand_topic[TOPIC_MAX], levels_topic[TOPIC_MAX], log_topic[TOPIC_MAX];
	char tx_topic[FDT_L_MAX][TOPIC_MAX];
	size_t raw_prefix_len;
	int64_t stats_us, next_stats;
	const struct mosquitto *data_client;
	int data_fd;
	ino_t data_ino;
	uint64_t publish_errors;
} g;

/* ------------------------------------------------------------ data bridge socket */

static int local_port(int fd)
{
	struct sockaddr_storage ss, peer;
	socklen_t l = sizeof ss;
	if (getsockname(fd, (struct sockaddr *)&ss, &l))
		return -1;
	l = sizeof peer;
	if (getpeername(fd, (struct sockaddr *)&peer, &l))
		return -1; /* listening socket */
	if (ss.ss_family == AF_INET)
		return ntohs(((struct sockaddr_in *)&ss)->sin_port);
	if (ss.ss_family == AF_INET6)
		return ntohs(((struct sockaddr_in6 *)&ss)->sin6_port);
	return -1;
}

/* the established socket on data_port that received data last (the CONNECT of the bridge) */
static void resolve_fd(void)
{
	g.data_fd = -1;
	if (!g.data_client)
		return;
	DIR *d = opendir("/proc/self/fd");
	if (!d)
		return;
	struct dirent *e;
	uint32_t best_age = UINT32_MAX;
	int best = -1;
	while ((e = readdir(d)) != NULL) {
		int fd = atoi(e->d_name);
		struct stat st;
		if (fd <= 2 || fd == dirfd(d) || fstat(fd, &st) || !S_ISSOCK(st.st_mode))
			continue;
		if (local_port(fd) != g.data_port)
			continue;
		struct tcp_info ti;
		socklen_t l = sizeof ti;
		if (getsockopt(fd, IPPROTO_TCP, TCP_INFO, &ti, &l) || ti.tcpi_state != TCP_ESTABLISHED)
			continue;
		if (ti.tcpi_last_data_recv < best_age) {
			best_age = ti.tcpi_last_data_recv;
			best = fd;
		}
	}
	closedir(d);
	if (best >= 0) {
		struct stat st;
		fstat(best, &st);
		g.data_fd = best;
		g.data_ino = st.st_ino;
	}
}

static int64_t data_outq(void *ctx)
{
	(void)ctx;
	struct stat st;
	if (!g.data_client)
		return -1;
	if (g.data_fd < 0 || fstat(g.data_fd, &st) || st.st_ino != g.data_ino)
		resolve_fd();
	int v = 0;
	if (g.data_fd < 0 || ioctl(g.data_fd, SIOCOUTQ, &v))
		return -1;
	return v;
}

/* ------------------------------------------------------------------- callbacks */

static int publish_fwd(void *ctx, int l, int qos, const uint8_t *payload, size_t len)
{
	(void)ctx;
	int rc = mosquitto_broker_publish_copy(NULL, g.tx_topic[l], (int)len, payload, qos, false, NULL);
	if (rc != MOSQ_ERR_SUCCESS)
		g.publish_errors++;
	return rc;
}

static int on_message_in(int event, void *event_data, void *userdata)
{
	struct mosquitto_evt_message *m = event_data;
	(void)event;
	(void)userdata;
	if (!strncmp(m->topic, g.raw_prefix, g.raw_prefix_len)) {
		const char *name = m->topic + g.raw_prefix_len;
		int l = fdt_dtp_index(&g.dtp, name, strlen(name));
		if (l >= 0)
			fdt_reg_on_raw(&g.reg, l, m->payload, m->payloadlen, fdt_now_us());
	} else if (!strcmp(m->topic, g.demand_topic)) {
		if (fdt_reg_on_demand(&g.reg, m->payload, m->payloadlen))
			mosquitto_log_printf(MOSQ_LOG_WARNING, "fdt_regulator: malformed demand table");
	} else if (!strcmp(m->topic, g.levels_topic)) {
		if (fdt_reg_on_levels(&g.reg, m->payload, m->payloadlen))
			mosquitto_log_printf(MOSQ_LOG_WARNING, "fdt_regulator: malformed priority table");
	}
	return MOSQ_ERR_SUCCESS;
}

static void log_stats(void)
{
	const struct fdt_reg_stats *s = &g.reg.st;
	mosquitto_log_printf(MOSQ_LOG_NOTICE,
		"fdt_regulator: steps=%llu raw=%llu fwd=%llu held=%llu drop=%llu stale=%llu no_d=%llu "
		"no_p=%llu bad_tables=%llu late_max=%lldus pub_err=%llu data_fd=%d",
		(unsigned long long)s->steps, (unsigned long long)s->raw, (unsigned long long)s->forwarded,
		(unsigned long long)s->withheld, (unsigned long long)s->dropped, (unsigned long long)s->stale,
		(unsigned long long)s->no_d, (unsigned long long)s->no_p, (unsigned long long)s->bad_tables,
		(long long)s->late_max_us, (unsigned long long)g.publish_errors, g.data_fd);
}

static int on_tick(int event, void *event_data, void *userdata)
{
	struct mosquitto_evt_tick *t = event_data;
	(void)event;
	(void)userdata;
	int64_t now = fdt_now_us(), next;
	struct fdt_reg_log lg;
	if (fdt_reg_tick(&g.reg, now, data_outq, NULL, &lg, &next) && g.log_topic[0]) {
		char buf[4096];
		size_t n = fdt_reg_log_json(&g.reg, &lg, buf, sizeof buf);
		mosquitto_broker_publish_copy(NULL, g.log_topic, (int)n, buf, 0, false, NULL);
	}
	if (g.stats_us > 0 && now >= g.next_stats) {
		log_stats();
		g.next_stats = now + g.stats_us;
	}
	int64_t d = next - fdt_now_us();
	long ms = d <= 1000 ? 1 : (long)((d + 999) / 1000);   /* the loop ignores a timeout of 0 */
	t->next_s = ms / 1000;
	t->next_ms = ms % 1000;
	return MOSQ_ERR_SUCCESS;
}

static int on_connect(int event, void *event_data, void *userdata)
{
	struct mosquitto_evt_connect *c = event_data;
	(void)event;
	(void)userdata;
	if (mosquitto_client_port(c->client) != g.data_port)
		return MOSQ_ERR_SUCCESS;
	g.data_client = c->client;
	resolve_fd();
	mosquitto_log_printf(MOSQ_LOG_NOTICE, "fdt_regulator: data bridge %s connected, fd=%d",
	                     mosquitto_client_address(c->client), g.data_fd);
	return MOSQ_ERR_SUCCESS;
}

static int on_disconnect(int event, void *event_data, void *userdata)
{
	struct mosquitto_evt_disconnect *d = event_data;
	(void)event;
	(void)userdata;
	if (d->client == g.data_client) {
		g.data_client = NULL;
		g.data_fd = -1;
		mosquitto_log_printf(MOSQ_LOG_NOTICE, "fdt_regulator: data bridge disconnected");
	}
	return MOSQ_ERR_SUCCESS;
}

/* ------------------------------------------------------------------------ init */

int mosquitto_plugin_init(mosquitto_plugin_id_t *identifier, void **userdata,
                          struct mosquitto_opt *opts, int opt_count)
{
	(void)userdata;
	memset(&g, 0, sizeof g);
	g.pid = identifier;
	g.data_fd = -1;
	g.data_port = 1884;
	g.max_msgs = 1024;
	g.stats_us = 10000000;
	const char *dtp_path = NULL, *log_topic = NULL;
	char err[256];
	for (int i = 0; i < opt_count; i++) {
		const char *k = opts[i].key, *v = opts[i].value;
		if (!strcmp(k, "dtp"))
			dtp_path = v;
		else if (!strcmp(k, "vessel"))
			g.vessel = atoi(v[0] == 'b' ? v + 1 : v);
		else if (!strcmp(k, "data_port"))
			g.data_port = atoi(v);
		else if (!strcmp(k, "max_msgs"))
			g.max_msgs = atoi(v);
		else if (!strcmp(k, "log_topic"))
			log_topic = v;
		else if (!strcmp(k, "stats_s"))
			g.stats_us = (int64_t)(atof(v) * 1e6);
		else
			mosquitto_log_printf(MOSQ_LOG_WARNING, "fdt_regulator: unknown option %s", k);
	}
	if (!dtp_path || g.vessel < 1) {
		mosquitto_log_printf(MOSQ_LOG_ERR, "fdt_regulator: plugin_opt_dtp and plugin_opt_vessel are required");
		return MOSQ_ERR_INVAL;
	}
	if (fdt_dtp_load(dtp_path, &g.dtp, err, sizeof err)) {
		mosquitto_log_printf(MOSQ_LOG_ERR, "fdt_regulator: %s", err);
		return MOSQ_ERR_INVAL;
	}
	snprintf(g.raw_prefix, sizeof g.raw_prefix, "boat/b%d/raw/", g.vessel);
	snprintf(g.demand_topic, sizeof g.demand_topic, "boat/b%d/demand", g.vessel);
	snprintf(g.levels_topic, sizeof g.levels_topic, "boat/b%d/levels", g.vessel);
	if (!log_topic)
		snprintf(g.log_topic, sizeof g.log_topic, "boat/b%d/regulator", g.vessel);
	else if (strcmp(log_topic, "none"))
		snprintf(g.log_topic, sizeof g.log_topic, "%s", log_topic);
	g.raw_prefix_len = strlen(g.raw_prefix);
	for (int l = 0; l < g.dtp.L; l++)
		snprintf(g.tx_topic[l], sizeof g.tx_topic[l], "boat/b%d/tx/%s", g.vessel, g.dtp.topic[l].name);
	if (fdt_reg_init(&g.reg, &g.dtp, g.max_msgs, fdt_now_us(), publish_fwd, NULL)) {
		mosquitto_log_printf(MOSQ_LOG_ERR, "fdt_regulator: out of memory");
		return MOSQ_ERR_NOMEM;
	}
	mosquitto_plugin_set_info(identifier, "fdt_regulator", "2.0");
	if (mosquitto_callback_register(identifier, MOSQ_EVT_MESSAGE_IN, on_message_in, NULL, NULL) ||
	    mosquitto_callback_register(identifier, MOSQ_EVT_TICK, on_tick, NULL, NULL) ||
	    mosquitto_callback_register(identifier, MOSQ_EVT_CONNECT, on_connect, NULL, NULL) ||
	    mosquitto_callback_register(identifier, MOSQ_EVT_DISCONNECT, on_disconnect, NULL, NULL))
		return MOSQ_ERR_UNKNOWN;
	mosquitto_log_printf(MOSQ_LOG_NOTICE,
		"fdt_regulator: vessel %d, L=%d, epoch=%.3f Ts=%.3f T1=%.3f T2=%.3f c_b=%.4f c_m=%.1f, "
		"data_port=%d, log=%s", g.vessel, g.dtp.L, g.dtp.epoch, g.dtp.Ts, g.dtp.T1, g.dtp.T2,
		g.dtp.c_b, g.dtp.c_m, g.data_port, g.log_topic[0] ? g.log_topic : "none");
	return MOSQ_ERR_SUCCESS;
}

int mosquitto_plugin_cleanup(void *userdata, struct mosquitto_opt *opts, int opt_count)
{
	(void)userdata;
	(void)opts;
	(void)opt_count;
	log_stats();
	fdt_reg_free(&g.reg);
	return MOSQ_ERR_SUCCESS;
}
