/*
 * Fleet-DT v2 vessel regulator and DTI program: Mosquitto 2.1 plugin (plugin API version 5) of the
 * vessel MQTT broker (paper Defs. demand, table, regulator; PR1, PR3; SPEC_v2.md Sections 4 and 6).
 * The logic lives in fdt_reg.c, the socket access in fdt_sock.c.
 *
 *   MOSQ_EVT_MESSAGE_IN   boat/b<i>/raw/<topic> (sources) and boat/b<i>/levels (AS, control bridge);
 *                         every message also continues to the local subscribers
 *   MOSQ_EVT_MESSAGE_OUT  each PUBLISH that the broker writes to a data bridge: QoS, exact packet
 *                         length and the stream offset (tcpi_bytes_acked + SIOCOUTQ before the write)
 *   MOSQ_EVT_TICK         phase 1 (demand, vessel log), v(t) (table), paced release every T_tick in
 *                         phase 3 (needs patch 0001, which keeps the tick deadline)
 *   MOSQ_EVT_CONNECT      finds the socket of a data bridge (listener port in data_ports)
 *   MOSQ_EVT_DISCONNECT   forgets it
 *
 * Timing of a release (Mosquitto 2.1.2): mosquitto_broker_publish() appends the message to
 * db.plugin_msgs and sets the loop timeout to 1 ms (src/plugin_public.c); the next iteration of
 * mosquitto_main_loop() (src/loop.c) runs queue_plugin_msgs() before plugin__handle_tick(), and
 * queue_plugin_msgs() -> db__messages_easy_queue() -> db__message_insert_outgoing() ->
 * db__message_write_inflight_out_latest() -> send__publish() (MOSQ_EVT_MESSAGE_OUT) ->
 * packet__queue() -> packet__write() writes the packet to the socket at once (lib/packet_mosq.c).
 * So the messages released in one tick reach the socket before the next tick reads SIOCOUTQNSD,
 * unless the socket is full (EAGAIN), in which case the broker keeps the rest in its out_packet
 * queue; the core counts these bytes from the MESSAGE_OUT offsets.
 *
 * mosquitto.conf of the vessel:
 *   listener 1883                               local programs, control bridge
 *   listener 1884                               data bridge (b2: also 1885, 1886)
 *   plugin /opt/fleet-dt/lib/fdt_regulator.so
 *   plugin_opt_dtp /etc/fleetdt/dtp_v2.json     DTP v2 (required)
 *   plugin_opt_vessel 1                         vessel index i (required)
 *   plugin_opt_mode fleetdt                     fleetdt | b1 | b2 | b3 | b5 | slotted | tdma
 *   plugin_opt_data_ports 1884                  data bridge listener(s); b2 default 1884,1885,1886
 *                                               (BE, AF41, EF)
 *   plugin_opt_max_msgs 4096                    stream messages per topic and step
 *   plugin_opt_stats_s 10                       statistics line in the broker log (0: off)
 */
#define _GNU_SOURCE
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#include <mosquitto.h>

#include "fdt_dtp2.h"
#include "fdt_reg.h"
#include "fdt_sock.h"

MOSQUITTO_PLUGIN_DECLARE_VERSION(5);

struct dsock {
	const struct mosquitto *client;
	int port, fd, v5;
	ino_t ino;
	char addr[64];
};

static struct {
	mosquitto_plugin_id_t *pid;
	struct fdt_dtp2 dtp;
	struct fdt_reg reg;
	int vessel, mode, n_ports;
	struct dsock ds[FDT_DATA_MAX];
	char raw_prefix[96], levels_topic[96];
	size_t raw_prefix_len;
	int64_t stats_us, next_stats;
	uint64_t sock_err;
} g;

/* ------------------------------------------------------------ data bridge sockets */

static int ds_fd(struct dsock *ds)
{
	if (!ds->client)
		return -1;
	if (ds->fd >= 0 && !fdt_sock_same(ds->fd, ds->ino))
		return ds->fd;
	ds->fd = fdt_sock_find(ds->port, ds->addr, &ds->ino);
	return ds->fd;
}

static void io_sock(void *ctx, struct fdt_sock *s)
{
	(void)ctx;
	int any = 0;
	memset(s, 0, sizeof *s);
	for (int k = 0; k < g.n_ports; k++) {
		struct fdt_sockinfo si;
		int fd = ds_fd(&g.ds[k]);
		if (fd < 0)
			continue;
		if (fdt_sock_read(fd, &si)) {
			g.sock_err++;
			continue;
		}
		any = 1;
		s->s[k].nsd = si.nsd;
		s->s[k].outq = si.outq;
		s->s[k].acked = si.acked;
		s->s[k].has_acked = si.has_acked;
	}
	s->n = any ? g.n_ports : 0;
}

static int io_publish(void *ctx, const char *topic, int qos, const void *payload, size_t len)
{
	(void)ctx;
	return mosquitto_broker_publish_copy(NULL, topic, (int)len, payload, qos, false, NULL);
}

/* ------------------------------------------------------------------- callbacks */

static int on_message_in(int event, void *event_data, void *userdata)
{
	struct mosquitto_evt_message *m = event_data;
	(void)event;
	(void)userdata;
	if (!m->topic)
		return MOSQ_ERR_SUCCESS;
	if (!strncmp(m->topic, g.raw_prefix, g.raw_prefix_len)) {
		const char *name = m->topic + g.raw_prefix_len;
		int l = fdt2_index(&g.dtp, name, strlen(name));
		if (l >= 0)
			fdt_reg_on_raw(&g.reg, l, m->payload, m->payloadlen, fdt_now_us());
	} else if (!strcmp(m->topic, g.levels_topic)) {
		if (fdt_reg_on_levels(&g.reg, m->payload, m->payloadlen, fdt_now_us()) < 0)
			mosquitto_log_printf(MOSQ_LOG_WARNING, "fdt_regulator: malformed table on %s", g.levels_topic);
	}
	return MOSQ_ERR_SUCCESS;
}

static int on_message_out(int event, void *event_data, void *userdata)
{
	struct mosquitto_evt_message *m = event_data;
	(void)event;
	(void)userdata;
	for (int k = 0; k < g.n_ports; k++) {
		struct dsock *ds = &g.ds[k];
		if (!ds->client || m->client != ds->client || !m->topic)
			continue;
		struct fdt_sockinfo si;
		int fd = ds_fd(ds), have = 0;
		uint64_t written = 0;
		if (fd >= 0 && !fdt_sock_read(fd, &si) && si.has_acked) {
			written = si.acked + (uint64_t)(si.outq > 0 ? si.outq : 0);
			have = 1;
		}
		/* MQTT 3.1.1 PUBLISH; MQTT 5 adds the property length (0 properties from the plugin) */
		uint32_t pkt = fdt2_publish_bytes(strlen(m->topic), m->qos, m->payloadlen) + (ds->v5 ? 1 : 0);
		fdt_reg_on_out(&g.reg, k, m->qos, pkt, m->payloadlen, written, have);
		break;
	}
	return MOSQ_ERR_SUCCESS;
}

static void log_stats(void)
{
	const struct fdt_reg_stats *s = &g.reg.st_;
	mosquitto_log_printf(MOSQ_LOG_NOTICE,
		"fdt_regulator: steps=%llu raw=%llu bad_raw=%llu rel=%llu disc=%llu wh=%llu inc=%llu over=%llu "
		"stale=%llu tab_ok=%llu tab_default=%llu tab_late=%llu tab_bad=%llu outs=%llu pub_err=%llu "
		"sock_err=%llu resync=%llu late_max=%lldus tick_max=%lldus fd=%d",
		(unsigned long long)s->steps, (unsigned long long)s->raw, (unsigned long long)s->bad_raw,
		(unsigned long long)s->released, (unsigned long long)s->discarded, (unsigned long long)s->withheld,
		(unsigned long long)s->inc, (unsigned long long)s->over, (unsigned long long)s->stale,
		(unsigned long long)s->tab_ok, (unsigned long long)s->tab_default, (unsigned long long)s->tab_late,
		(unsigned long long)s->tab_bad, (unsigned long long)s->outs, (unsigned long long)s->pub_err,
		(unsigned long long)g.sock_err, (unsigned long long)s->resync, (long long)s->late_max_us, (long long)s->tick_us_max, g.ds[0].fd);
}

static int on_tick(int event, void *event_data, void *userdata)
{
	struct mosquitto_evt_tick *t = event_data;
	(void)event;
	(void)userdata;
	int64_t now = fdt_now_us();
	int64_t next = fdt_reg_tick(&g.reg, now);
	if (g.stats_us > 0 && now >= g.next_stats) {
		log_stats();
		g.next_stats = now + g.stats_us;
	}
	/* the loop takes whole milliseconds and ignores a timeout of 0; round up so that a tick never
	 * comes before its instant */
	int64_t dly = next - fdt_now_us();
	long ms = dly <= 1000 ? 1 : (long)((dly + 999) / 1000);
	if (ms > 1000)
		ms = 1000;
	t->next_s = ms / 1000;
	t->next_ms = ms % 1000;
	return MOSQ_ERR_SUCCESS;
}

static int on_connect(int event, void *event_data, void *userdata)
{
	struct mosquitto_evt_connect *c = event_data;
	(void)event;
	(void)userdata;
	int port = mosquitto_client_port(c->client);
	for (int k = 0; k < g.n_ports; k++) {
		struct dsock *ds = &g.ds[k];
		if (ds->port != port)
			continue;
		const char *a = mosquitto_client_address(c->client);
		ds->client = c->client;
		ds->v5 = mosquitto_client_protocol_version(c->client) == 5;
		snprintf(ds->addr, sizeof ds->addr, "%s", a ? a : "");
		ds->fd = fdt_sock_find(ds->port, ds->addr, &ds->ino);
		fdt_reg_sock_reset(&g.reg, k);
		mosquitto_log_printf(MOSQ_LOG_NOTICE, "fdt_regulator: data bridge %s on port %d connected, fd=%d%s",
		                     ds->addr, port, ds->fd, ds->v5 ? " (MQTT 5: use bridge_protocol_version mqttv311)" : "");
	}
	return MOSQ_ERR_SUCCESS;
}

static int on_disconnect(int event, void *event_data, void *userdata)
{
	struct mosquitto_evt_disconnect *d = event_data;
	(void)event;
	(void)userdata;
	for (int k = 0; k < g.n_ports; k++)
		if (d->client == g.ds[k].client) {
			g.ds[k].client = NULL;
			g.ds[k].fd = -1;
			fdt_reg_sock_reset(&g.reg, k);
			mosquitto_log_printf(MOSQ_LOG_NOTICE, "fdt_regulator: data bridge on port %d disconnected", g.ds[k].port);
		}
	return MOSQ_ERR_SUCCESS;
}

/* ------------------------------------------------------------------------ init */

static int parse_ports(const char *v)
{
	char buf[128], *save = NULL;
	snprintf(buf, sizeof buf, "%s", v);
	g.n_ports = 0;
	for (char *p = strtok_r(buf, ", ", &save); p; p = strtok_r(NULL, ", ", &save)) {
		int port = atoi(p);
		if (port < 1 || port > 65535 || g.n_ports >= FDT_DATA_MAX)
			return -1;
		g.ds[g.n_ports++].port = port;
	}
	return g.n_ports ? 0 : -1;
}

int mosquitto_plugin_init(mosquitto_plugin_id_t *identifier, void **userdata,
                          struct mosquitto_opt *opts, int opt_count)
{
	(void)userdata;
	memset(&g, 0, sizeof g);
	g.pid = identifier;
	g.stats_us = 10000000;
	g.mode = FDT_MODE_FLEETDT;
	for (int k = 0; k < FDT_DATA_MAX; k++)
		g.ds[k].fd = -1;
	const char *dtp_path = NULL, *ports = NULL;
	int max_msgs = 4096;
	char err[256];
	for (int i = 0; i < opt_count; i++) {
		const char *k = opts[i].key, *v = opts[i].value;
		if (!strcmp(k, "dtp"))
			dtp_path = v;
		else if (!strcmp(k, "vessel"))
			g.vessel = atoi(v[0] == 'b' ? v + 1 : v);
		else if (!strcmp(k, "mode")) {
			g.mode = fdt_mode_parse(v);
			if (g.mode < 0) {
				mosquitto_log_printf(MOSQ_LOG_ERR, "fdt_regulator: unknown mode %s", v);
				return MOSQ_ERR_INVAL;
			}
		} else if (!strcmp(k, "data_ports") || !strcmp(k, "data_port"))
			ports = v;
		else if (!strcmp(k, "max_msgs"))
			max_msgs = atoi(v);
		else if (!strcmp(k, "stats_s"))
			g.stats_us = (int64_t)(atof(v) * 1e6);
		else
			mosquitto_log_printf(MOSQ_LOG_WARNING, "fdt_regulator: unknown option %s", k);
	}
	if (!ports)
		ports = g.mode == FDT_MODE_B2 ? "1884,1885,1886" : "1884";
	if (!dtp_path || g.vessel < 1 || parse_ports(ports)) {
		mosquitto_log_printf(MOSQ_LOG_ERR,
			"fdt_regulator: plugin_opt_dtp and plugin_opt_vessel are required, data_ports a list of up to %d ports",
			FDT_DATA_MAX);
		return MOSQ_ERR_INVAL;
	}
	if (fdt2_dtp_load(dtp_path, &g.dtp, err, sizeof err)) {
		mosquitto_log_printf(MOSQ_LOG_ERR, "fdt_regulator: %s", err);
		return MOSQ_ERR_INVAL;
	}
	snprintf(g.raw_prefix, sizeof g.raw_prefix, "boat/b%d/raw/", g.vessel);
	snprintf(g.levels_topic, sizeof g.levels_topic, "boat/b%d/levels", g.vessel);
	g.raw_prefix_len = strlen(g.raw_prefix);
	struct fdt_io io = {NULL, io_publish, io_sock};
	if (fdt_reg_init(&g.reg, &g.dtp, g.vessel, g.mode, max_msgs, fdt_now_us(), &io)) {
		mosquitto_log_printf(MOSQ_LOG_ERR, "fdt_regulator: out of memory");
		return MOSQ_ERR_NOMEM;
	}
	mosquitto_plugin_set_info(identifier, "fdt_regulator", "v2");
	if (mosquitto_callback_register(identifier, MOSQ_EVT_MESSAGE_IN, on_message_in, NULL, NULL) ||
	    mosquitto_callback_register(identifier, MOSQ_EVT_MESSAGE_OUT, on_message_out, NULL, NULL) ||
	    mosquitto_callback_register(identifier, MOSQ_EVT_TICK, on_tick, NULL, NULL) ||
	    mosquitto_callback_register(identifier, MOSQ_EVT_CONNECT, on_connect, NULL, NULL) ||
	    mosquitto_callback_register(identifier, MOSQ_EVT_DISCONNECT, on_disconnect, NULL, NULL))
		return MOSQ_ERR_UNKNOWN;
	mosquitto_log_printf(MOSQ_LOG_NOTICE,
		"fdt_regulator v2: vessel %d, mode %s, L=%d, epoch=%.3f Ts=%.3f T1=%.3f T2=%.3f T_tick=%.4f L_w=%lld, "
		"data_ports=%s", g.vessel, fdt_mode_name(g.mode), g.dtp.L, g.dtp.epoch, g.dtp.Ts, g.dtp.T1, g.dtp.T2,
		g.dtp.T_tick, (long long)g.dtp.L_w_bytes, ports);
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
