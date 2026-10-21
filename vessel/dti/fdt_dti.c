/*
 * Fleet-DT DTI program of a vessel (Definition 9, R3): at start(t) + offset of each
 * step t it publishes the demand table D_i(t) = (t, L, Q_i(t), d_{i,1}(t) .. d_{i,L}(t)) on
 * boat/b<i>/demand (QoS 0) of the vessel broker; the control bridge carries it to
 * the GS before the end of phase 1.
 *
 * Q_i(t) = ceil(c_b o_i(t) + c_k n_i(t)) is the backlog of the vessel at the start of phase 1:
 * o_i = SIOCOUTQ of the TCP socket of the data bridge and n_i = PUBLISH packets with at least one
 * byte among those o_i bytes. That socket belongs to the vessel broker, and this program, another
 * process, can neither ioctl it nor see the packets written to it, so it writes Q_i = 0 and the regulator plugin
 * (plugin_opt_demand external) overwrites the field in place when the table enters the broker
 * (fdt_regulator_plugin.c, fdt_preg_stamp_demand; SPEC.md Section 5).
 *
 * d_{i,l}(t) = ceil(c_b (x_l + 6 n_l) + c_m,l n_l), n_l = ceil(r_l (Ts + J_l)), x_l = n_l s_l
 * (SPEC.md Section 5) for each topic l that published a raw message in the last
 * --idle steps, and 0 for the others (a topic whose publisher is down claims no
 * part of the channel). With --idle 0 every topic claims its demand.
 *
 * The DTP reader, the cost bound and d_{i,l} are those of the vessel regulator in mode paper
 * (common/fdt_paper.c: fdt_pdtp_load, fdt_p_demand), so both programs compute the same table from
 * the same file: J_l is the "jitter" of topic l in the DTP, else --jitter-ms J (default 0); c_b and
 * c_m come from the file, else from the cost bound of the paper. The vessel regulator with
 * plugin_opt_demand external polices each topic to the d_{i,l}(t) published here, so the jitter
 * allowance has to be inside this value: pass the plugin_opt_jitter_ms of the regulator (default 2)
 * as --jitter-ms (emu/fleet.py and fleetdt/gsconf.py do).
 *
 * The publication instant follows clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME)
 * under SCHED_FIFO (when the process holds CAP_SYS_NICE), so it does not drift.
 *
 *   fdt_dti --dtp /etc/fleetdt/dtp.json --vessel 1 [--broker 127.0.0.1:1883]
 *           [--offset-ms 1] [--idle 8] [--jitter-ms 0] [--prio 50] [--steps N] [--verbose]
 */
#define _GNU_SOURCE
#include <errno.h>
#include <getopt.h>
#include <inttypes.h>
#include <math.h>
#include <sched.h>
#include <signal.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include <mosquitto.h>

#include "fdt_common.h"
#include "fdt_paper.h"

static struct fdt_pdtp dtp;
static int vessel = 0;
static char raw_prefix[96];
static size_t raw_prefix_len;
/* last step with a raw message of topic l; a mutex instead of 64-bit atomics, which need
 * libatomic on the armv6 baseline of Raspbian */
static int64_t last_raw[FDT_L_MAX];
static pthread_mutex_t raw_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile sig_atomic_t stop;

static void on_signal(int s)
{
	(void)s;
	stop = 1;
}

static void on_connect(struct mosquitto *m, void *u, int rc)
{
	(void)u;
	if (rc == 0) {
		char sub[128];
		snprintf(sub, sizeof sub, "%s+", raw_prefix);
		mosquitto_subscribe(m, NULL, sub, 0);
	}
}

static void on_message(struct mosquitto *m, void *u, const struct mosquitto_message *msg)
{
	(void)m;
	(void)u;
	if (strncmp(msg->topic, raw_prefix, raw_prefix_len))
		return;
	const char *name = msg->topic + raw_prefix_len;
	int l = fdt_pdtp_index(&dtp, name, strlen(name));
	if (l >= 0) {
		int64_t t = fdt_p_collect_step(&dtp, fdt_now_us());
		pthread_mutex_lock(&raw_lock);
		last_raw[l] = t;
		pthread_mutex_unlock(&raw_lock);
	}
}

static void usage(void)
{
	fprintf(stderr, "usage: fdt_dti --dtp FILE --vessel I [--broker HOST:PORT] [--offset-ms MS] "
	                "[--idle STEPS] [--jitter-ms MS] [--prio P] [--steps N] [--verbose]\n");
	exit(2);
}

int main(int argc, char **argv)
{
	const char *dtp_path = NULL;
	char host[128] = "127.0.0.1";
	int port = 1883, idle = 8, prio = 50, verbose = 0;
	double offset_ms = 1.0, jitter_ms = 0.0;
	long long steps = -1;
	static const struct option lo[] = {
		{"dtp", 1, 0, 'd'}, {"vessel", 1, 0, 'v'}, {"broker", 1, 0, 'b'}, {"offset-ms", 1, 0, 'o'},
		{"idle", 1, 0, 'i'}, {"jitter-ms", 1, 0, 'j'}, {"prio", 1, 0, 'p'}, {"steps", 1, 0, 'n'},
		{"verbose", 0, 0, 'V'},
		{"help", 0, 0, 'h'}, {0, 0, 0, 0}};
	int c;
	while ((c = getopt_long(argc, argv, "d:v:b:o:i:j:p:n:Vh", lo, NULL)) != -1) {
		switch (c) {
		case 'd': dtp_path = optarg; break;
		case 'v': vessel = atoi(optarg[0] == 'b' ? optarg + 1 : optarg); break;
		case 'b': {
			char *colon = strrchr(optarg, ':');
			if (colon) {
				snprintf(host, sizeof host, "%.*s", (int)(colon - optarg), optarg);
				port = atoi(colon + 1);
			} else {
				snprintf(host, sizeof host, "%s", optarg);
			}
			break;
		}
		case 'o': offset_ms = atof(optarg); break;
		case 'i': idle = atoi(optarg); break;
		case 'j': jitter_ms = atof(optarg); break;
		case 'p': prio = atoi(optarg); break;
		case 'n': steps = atoll(optarg); break;
		case 'V': verbose = 1; break;
		default: usage();
		}
	}
	if (!dtp_path || vessel < 1)
		usage();
	char err[256];
	if (fdt_pdtp_load(dtp_path, &dtp, err, sizeof err)) {
		fprintf(stderr, "fdt_dti: %s\n", err);
		return 1;
	}
	int64_t offset_us = (int64_t)(offset_ms * 1000.0);
	if (offset_us < 0 || offset_us >= dtp.T1_us) {
		fprintf(stderr, "fdt_dti: the offset must lie in [0, T1)\n");
		return 2;
	}
	snprintf(raw_prefix, sizeof raw_prefix, "boat/b%d/raw/", vessel);
	raw_prefix_len = strlen(raw_prefix);
	char topic[96];
	snprintf(topic, sizeof topic, "boat/b%d/demand", vessel);
	uint32_t dmax[FDT_L_MAX];
	if (jitter_ms < 0) {
		fprintf(stderr, "fdt_dti: the jitter must be >= 0\n");
		return 2;
	}
	for (int l = 0; l < dtp.L; l++) {
		/* J_l: the jitter of the topic in the DTP, else --jitter-ms (the rule of fdt_preg_init) */
		double J = dtp.topic[l].has_jitter ? dtp.topic[l].jitter : jitter_ms * 1e-3;
		dmax[l] = fdt_p_demand(&dtp, l, J);
		last_raw[l] = INT64_MIN / 2;
	}

	struct sigaction sa;
	memset(&sa, 0, sizeof sa);
	sa.sa_handler = on_signal;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

	mosquitto_lib_init();
	char cid[64];
	snprintf(cid, sizeof cid, "fdt-dti-b%d-%d", vessel, (int)getpid());
	struct mosquitto *m = mosquitto_new(cid, true, NULL);
	if (!m) {
		fprintf(stderr, "fdt_dti: mosquitto_new failed\n");
		return 1;
	}
	mosquitto_connect_callback_set(m, on_connect);
	mosquitto_message_callback_set(m, on_message);
	mosquitto_reconnect_delay_set(m, 1, 4, false);
	mosquitto_max_inflight_messages_set(m, 0);
	int rc = mosquitto_connect(m, host, port, 10);
	if (rc != MOSQ_ERR_SUCCESS)
		fprintf(stderr, "fdt_dti: connect %s:%d: %s (retrying)\n", host, port, mosquitto_strerror(rc));
	mosquitto_loop_start(m);
	if (rc != MOSQ_ERR_SUCCESS)
		mosquitto_reconnect_async(m);

	/* real-time scheduling of this thread (the network thread keeps SCHED_OTHER) */
	struct sched_param sp = {.sched_priority = prio};
	if (prio > 0 && sched_setscheduler(0, SCHED_FIFO, &sp))
		fprintf(stderr, "fdt_dti: SCHED_FIFO unavailable (%s), running under SCHED_OTHER\n", strerror(errno));
	mlockall(MCL_CURRENT | MCL_FUTURE);

	fprintf(stderr, "fdt_dti: vessel %d, L=%d, epoch=%.3f, Ts=%.3f s, offset=%.3f ms, idle=%d steps, jitter=%.3f ms, "
	        "broker %s:%d\n", vessel, dtp.L, dtp.epoch, dtp.Ts, offset_ms, idle, jitter_ms, host, port);
	int64_t t = fdt_p_step_at(&dtp, fdt_now_us()) + 1;
	uint64_t published = 0, late = 0;
	int64_t late_max = 0;
	while (!stop && steps != 0) {
		int64_t at = fdt_p_start_us(&dtp, t) + offset_us;
		fdt_sleep_until(at);
		if (stop)
			break;
		int64_t now = fdt_now_us();
		if (now - fdt_p_start_us(&dtp, t) >= dtp.T1_us) {   /* missed phase 1 of step t */
			late++;
			t = fdt_p_step_at(&dtp, now) + 1;
			continue;
		}
		if (now - at > late_max)
			late_max = now - at;
		uint32_t d[FDT_L_MAX];
		for (int l = 0; l < dtp.L; l++) {
			pthread_mutex_lock(&raw_lock);
			int64_t seen = last_raw[l];
			pthread_mutex_unlock(&raw_lock);
			d[l] = (idle <= 0 || t - seen <= idle) ? dmax[l] : 0;
		}
		uint8_t buf[FDT_DEMAND_HDR + 4 * FDT_L_MAX];
		/* Q_i = 0: the regulator plugin writes the backlog when the table enters the vessel broker */
		size_t n = fdt_demand_encode(buf, (uint32_t)t, 0, dtp.L, d);
		rc = mosquitto_publish(m, NULL, topic, (int)n, buf, 0, false);
		if (rc == MOSQ_ERR_SUCCESS)
			published++;
		if (verbose) {
			fprintf(stderr, "t=%" PRId64 " +%" PRId64 "us d=", t, now - fdt_p_start_us(&dtp, t));
			for (int l = 0; l < dtp.L; l++)
				fprintf(stderr, "%s%u", l ? "," : "", d[l]);
			fprintf(stderr, " rc=%d\n", rc);
		}
		t++;
		if (steps > 0)
			steps--;
	}
	fprintf(stderr, "fdt_dti: published=%" PRIu64 " missed_phase1=%" PRIu64 " max_delay=%" PRId64 "us\n",
	        published, late, late_max);
	usleep(200000);   /* lets the network thread send the last table */
	mosquitto_disconnect(m);
	mosquitto_loop_stop(m, false);
	mosquitto_destroy(m);
	mosquitto_lib_cleanup();
	return 0;
}
