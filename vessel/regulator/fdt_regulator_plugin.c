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
 *   plugin_opt_p1_spacing_us -1                 phase 1 (demand) of the vessel i (position i - 1 of the DTP
 *                                               list, i = 1..M) starts i * (T1 - g1) / (M + 1) after start(t)
 *                                               (-1), or i * spacing (>= 0, override)
 *   plugin_opt_p1_guard_us -1                   g1: -1 = eps + 2 T_tick of the DTP (the rule); >= 0 overrides
 *   plugin_opt_stats_s 10                       statistics line in the broker log (0: off)
 *
 * Mode paper (and its ablations paper_nopolice = B5a, paper_noeos = B5b) runs the core of
 * fdt_reg_paper.c instead: the "Vessel regulator" Definition of the paper with the binary wire formats
 * of fleetdt/model.py and the 0-based step clock of fleetdt/as_service.py. It reads the DTP with
 * fdt_paper.c (dtp.json of SPEC.md or a DTP v2 file) and ignores p1_spacing_us and p1_guard_us.
 *   plugin_opt_mode paper                       paper | paper_nopolice | paper_noeos
 *   plugin_opt_dtp /etc/fleetdt/dtp.json        DTP (required)
 *   plugin_opt_demand internal                  internal: the regulator publishes D_i(t) on boat/b<i>/demand;
 *                                               external: the DTI program fdt_dti publishes it and the
 *                                               regulator polices to the table it sees on that topic
 *   plugin_opt_log /var/log/fleetdt/reg-b1.jsonl  vessel log: one JSON line per message not forwarded
 *                                               (step, topic, seq, bytes, reason); absent: no file
 *   plugin_opt_jitter_ms 2                      J_l of the topics without "jitter" in the DTP (internal demand)
 *   plugin_opt_idle 8                           steps without raw messages after which d_l = 0 (0: never)
 *   plugin_opt_p1_offset_us 0                   demand at start(t) + offset (default 0: no offset)
 *   plugin_opt_max_msgs 4096                    messages per topic and step
 *
 * Baselines of the paper stack in the same core (fdt_reg_paper.h): paper_b1 (B1, B10), paper_b2 (B2),
 * paper_b3 (B3), b8 (B8; alias paper_b8) and paper_tdma (B9). The names b1, b2, b3 and tdma stay with the
 * older core above (as2 stack).
 *   plugin_opt_feedback_topic fleet/feedback    fleet feedback of the GS (paper_b3, b8, paper_tdma)
 *   plugin_opt_U 0.8                            b8: U before the first feedback that carries one
 *                                               (default: "U" of the DTP)
 *   plugin_opt_lw_bytes 16384                   paper_tdma: unsent bytes of the data bridge plus bytes
 *                                               released in one tick
 *   plugin_opt_tick_us 1000                     paper_tdma: release period inside the slot
 *   plugin_opt_data_ports                       paper_b2 default 1884,1885,1886
 */
#define _GNU_SOURCE
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#include <mosquitto.h>

#include "fdt_dtp2.h"
#include "fdt_paper.h"
#include "fdt_reg.h"
#include "fdt_reg_paper.h"
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
	/* mode paper */
	int paper;
	struct fdt_pdtp pdtp;
	struct fdt_preg preg;
	FILE *plog;
	char demand_topic[96], feedback_topic[96];
	int warned_foreign;
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
	if (g.paper) {
		int64_t now = fdt_now_us();
		if (!strncmp(m->topic, g.raw_prefix, g.raw_prefix_len)) {
			const char *name = m->topic + g.raw_prefix_len;
			int l = fdt_pdtp_index(&g.pdtp, name, strlen(name));
			if (l >= 0)
				fdt_preg_on_raw(&g.preg, l, m->payload, m->payloadlen, now);
		} else if (!strcmp(m->topic, g.levels_topic)) {
			if (fdt_preg_on_levels(&g.preg, m->payload, m->payloadlen, now) < 0)
				mosquitto_log_printf(MOSQ_LOG_WARNING, "fdt_regulator: malformed priority table on %s", g.levels_topic);
		} else if (!strcmp(m->topic, g.feedback_topic)) {
			if (fdt_preg_on_feedback(&g.preg, m->payload, m->payloadlen, now) < 0)
				mosquitto_log_printf(MOSQ_LOG_WARNING, "fdt_regulator: malformed feedback on %s", g.feedback_topic);
		} else if (!strcmp(m->topic, g.demand_topic)) {
			int rc = fdt_preg_on_demand(&g.preg, m->payload, m->payloadlen, now);
			if (rc == 2 && !g.warned_foreign) {
				g.warned_foreign = 1;
				mosquitto_log_printf(MOSQ_LOG_WARNING, "fdt_regulator: another program publishes %s; with "
				                     "plugin_opt_demand internal the regulator polices only its own table",
				                     g.demand_topic);
			} else if (rc < 0) {
				mosquitto_log_printf(MOSQ_LOG_WARNING, "fdt_regulator: malformed demand table on %s", g.demand_topic);
			}
		}
		return MOSQ_ERR_SUCCESS;
	}
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
	if (g.paper)
		return MOSQ_ERR_SUCCESS;
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

static void log_stats_paper(void)
{
	const struct fdt_preg_stats *s = &g.preg.s;
	mosquitto_log_printf(MOSQ_LOG_NOTICE,
		"fdt_regulator (%s): steps=%llu raw=%llu fwd=%llu late_fwd=%llu logged=%llu (no_table=%llu no_demand=%llu "
		"level0=%llu police=%llu overflow=%llu) tab_ok=%llu tab_none=%llu tab_late=%llu tab_bad=%llu dem_pub=%llu "
		"dem_ext=%llu dem_late=%llu dem_bad=%llu dem_foreign=%llu resync=%llu pub_err=%llu log_err=%llu "
		"sock_err=%llu late_max=%lldus fb_ok=%llu fb_old=%llu fb_bad=%llu budget=%llu slot=%llu",
		fdt_pmode_name(g.preg.mode), (unsigned long long)s->steps, (unsigned long long)s->raw,
		(unsigned long long)s->fwd, (unsigned long long)s->late_fwd, (unsigned long long)s->logged,
		(unsigned long long)s->reason[FDT_PR_NO_TABLE], (unsigned long long)s->reason[FDT_PR_NO_DEMAND],
		(unsigned long long)s->reason[FDT_PR_LEVEL0], (unsigned long long)s->reason[FDT_PR_POLICE],
		(unsigned long long)s->overflow, (unsigned long long)s->tab_ok, (unsigned long long)s->tab_none,
		(unsigned long long)s->tab_late, (unsigned long long)s->tab_bad, (unsigned long long)s->dem_pub,
		(unsigned long long)s->dem_ext, (unsigned long long)s->dem_late, (unsigned long long)s->dem_bad,
		(unsigned long long)s->dem_foreign, (unsigned long long)s->resync, (unsigned long long)s->pub_err,
		(unsigned long long)s->log_err, (unsigned long long)g.sock_err, (long long)s->late_max_us,
		(unsigned long long)s->fb_ok, (unsigned long long)s->fb_old, (unsigned long long)s->fb_bad,
		(unsigned long long)s->reason[FDT_PR_BUDGET], (unsigned long long)s->reason[FDT_PR_SLOT]);
}

static void log_stats(void)
{
	if (g.paper) {
		log_stats_paper();
		return;
	}
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
	int64_t next = g.paper ? fdt_preg_tick(&g.preg, now) : fdt_reg_tick(&g.reg, now);
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
		if (!g.paper)
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
			if (!g.paper)
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

static int register_callbacks(mosquitto_plugin_id_t *identifier)
{
	if (mosquitto_callback_register(identifier, MOSQ_EVT_MESSAGE_IN, on_message_in, NULL, NULL) ||
	    mosquitto_callback_register(identifier, MOSQ_EVT_MESSAGE_OUT, on_message_out, NULL, NULL) ||
	    mosquitto_callback_register(identifier, MOSQ_EVT_TICK, on_tick, NULL, NULL) ||
	    mosquitto_callback_register(identifier, MOSQ_EVT_CONNECT, on_connect, NULL, NULL) ||
	    mosquitto_callback_register(identifier, MOSQ_EVT_DISCONNECT, on_disconnect, NULL, NULL))
		return -1;
	return 0;
}

static int init_paper(mosquitto_plugin_id_t *identifier, const char *dtp_path, const char *ports,
                      const char *log_path, int max_msgs, struct fdt_preg_opts *po, const char *fb_topic)
{
	char err[256];
	if (fdt_pdtp_load(dtp_path, &g.pdtp, err, sizeof err)) {
		mosquitto_log_printf(MOSQ_LOG_ERR, "fdt_regulator: %s", err);
		return MOSQ_ERR_INVAL;
	}
	if (log_path) {
		g.plog = fopen(log_path, "a");
		if (!g.plog) {
			mosquitto_log_printf(MOSQ_LOG_ERR, "fdt_regulator: cannot open the vessel log %s", log_path);
			return MOSQ_ERR_INVAL;
		}
	}
	po->max_msgs = max_msgs;
	po->log = g.plog;
	snprintf(g.raw_prefix, sizeof g.raw_prefix, "boat/b%d/raw/", g.vessel);
	snprintf(g.levels_topic, sizeof g.levels_topic, "boat/b%d/levels", g.vessel);
	snprintf(g.demand_topic, sizeof g.demand_topic, "boat/b%d/demand", g.vessel);
	snprintf(g.feedback_topic, sizeof g.feedback_topic, "%s", fb_topic ? fb_topic : "fleet/feedback");
	g.raw_prefix_len = strlen(g.raw_prefix);
	struct fdt_io io = {NULL, io_publish, io_sock};
	if (fdt_preg_init(&g.preg, &g.pdtp, g.vessel, po, fdt_now_us(), &io)) {
		mosquitto_log_printf(MOSQ_LOG_ERR, "fdt_regulator: out of memory");
		return MOSQ_ERR_NOMEM;
	}
	mosquitto_plugin_set_info(identifier, "fdt_regulator", "paper");
	if (register_callbacks(identifier))
		return MOSQ_ERR_UNKNOWN;
	mosquitto_log_printf(MOSQ_LOG_NOTICE,
		"fdt_regulator: vessel %d, mode %s, demand %s, L=%d, epoch=%.3f Ts=%.3f T1=%.3f T2=%.3f, c_b=%.4f, "
		"data_ports=%s, phase 1 offset %lld us, vessel log %s, jitter %.3f ms%s%s", g.vessel, fdt_pmode_name(po->mode),
		po->demand_ext ? "external" : "internal", g.pdtp.L, g.pdtp.epoch, g.pdtp.Ts, g.pdtp.T1, g.pdtp.T2,
		g.pdtp.c_b, ports, (long long)g.preg.p1_off_us, log_path ? log_path : "off", po->jitter_s * 1e3,
		fdt_pmode_table(po->mode) ? "" : ", feedback on ", fdt_pmode_table(po->mode) ? "" : g.feedback_topic);
	return MOSQ_ERR_SUCCESS;
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
	const char *dtp_path = NULL, *ports = NULL, *log_path = NULL, *fb_topic = NULL;
	int max_msgs = 4096;
	long long p1_spacing = -1, p1_guard = -1;
	struct fdt_preg_opts po;
	fdt_preg_opts_default(&po);
	char err[256];
	for (int i = 0; i < opt_count; i++) {
		const char *k = opts[i].key, *v = opts[i].value;
		if (!strcmp(k, "dtp"))
			dtp_path = v;
		else if (!strcmp(k, "vessel"))
			g.vessel = atoi(v[0] == 'b' ? v + 1 : v);
		else if (!strcmp(k, "mode")) {
			int pm = fdt_pmode_parse(v);
			if (pm >= 0) {
				g.paper = 1;
				po.mode = pm;
				continue;
			}
			g.paper = 0;
			g.mode = fdt_mode_parse(v);
			if (g.mode < 0) {
				mosquitto_log_printf(MOSQ_LOG_ERR, "fdt_regulator: unknown mode %s", v);
				return MOSQ_ERR_INVAL;
			}
		} else if (!strcmp(k, "data_ports") || !strcmp(k, "data_port"))
			ports = v;
		else if (!strcmp(k, "max_msgs"))
			max_msgs = atoi(v);
		else if (!strcmp(k, "p1_spacing_us"))
			p1_spacing = atoll(v);
		else if (!strcmp(k, "p1_guard_us"))
			p1_guard = atoll(v);
		else if (!strcmp(k, "stats_s"))
			g.stats_us = (int64_t)(atof(v) * 1e6);
		else if (!strcmp(k, "demand")) {
			if (strcmp(v, "internal") && strcmp(v, "external")) {
				mosquitto_log_printf(MOSQ_LOG_ERR, "fdt_regulator: plugin_opt_demand must be internal or external");
				return MOSQ_ERR_INVAL;
			}
			po.demand_ext = !strcmp(v, "external");
		} else if (!strcmp(k, "log"))
			log_path = v;
		else if (!strcmp(k, "jitter_ms"))
			po.jitter_s = atof(v) * 1e-3;
		else if (!strcmp(k, "idle"))
			po.idle = atoi(v);
		else if (!strcmp(k, "p1_offset_us"))
			po.p1_off_us = atoll(v);
		else if (!strcmp(k, "feedback_topic"))
			fb_topic = v;
		else if (!strcmp(k, "U"))
			po.U = atof(v);
		else if (!strcmp(k, "lw_bytes"))
			po.lw_bytes = atoll(v);
		else if (!strcmp(k, "tick_us"))
			po.tick_us = atoll(v);
		else
			mosquitto_log_printf(MOSQ_LOG_WARNING, "fdt_regulator: unknown option %s", k);
	}
	if (!ports)
		ports = (!g.paper && g.mode == FDT_MODE_B2) || (g.paper && po.mode == FDT_PMODE_B2) ? "1884,1885,1886" : "1884";
	if (!dtp_path || g.vessel < 1 || parse_ports(ports)) {
		mosquitto_log_printf(MOSQ_LOG_ERR,
			"fdt_regulator: plugin_opt_dtp and plugin_opt_vessel are required, data_ports a list of up to %d ports",
			FDT_DATA_MAX);
		return MOSQ_ERR_INVAL;
	}
	if (g.paper)
		return init_paper(identifier, dtp_path, ports, log_path, max_msgs, &po, fb_topic);
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
	fdt_reg_set_p1_offset_guard(&g.reg, p1_spacing, p1_guard);
	mosquitto_plugin_set_info(identifier, "fdt_regulator", "v2");
	if (register_callbacks(identifier))
		return MOSQ_ERR_UNKNOWN;
	mosquitto_log_printf(MOSQ_LOG_NOTICE,
		"fdt_regulator v2: vessel %d, mode %s, L=%d, epoch=%.3f Ts=%.3f T1=%.3f T2=%.3f T_tick=%.4f L_w=%lld, "
		"data_ports=%s, phase 1 offset %lld us", g.vessel, fdt_mode_name(g.mode), g.dtp.L, g.dtp.epoch, g.dtp.Ts,
		g.dtp.T1, g.dtp.T2, g.dtp.T_tick, (long long)g.dtp.L_w_bytes, ports, (long long)g.reg.p1_off_us);
	return MOSQ_ERR_SUCCESS;
}

int mosquitto_plugin_cleanup(void *userdata, struct mosquitto_opt *opts, int opt_count)
{
	(void)userdata;
	(void)opts;
	(void)opt_count;
	log_stats();
	if (g.paper) {
		fdt_preg_free(&g.preg);
		if (g.plog)
			fclose(g.plog);
		g.plog = NULL;
	} else {
		fdt_reg_free(&g.reg);
	}
	return MOSQ_ERR_SUCCESS;
}
