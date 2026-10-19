#define _GNU_SOURCE
#include "fdt_sock.h"

#include <arpa/inet.h>
#include <dirent.h>
#include <netinet/in.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

/* the kernel's struct tcp_info (with tcpi_bytes_acked), not the older copy of the C library */
#include <linux/sockios.h>
#include <linux/tcp.h>

#ifndef SIOCOUTQNSD
#define SIOCOUTQNSD 0x894B
#endif
#ifndef SIOCOUTQ
#define SIOCOUTQ 0x5411   /* TIOCOUTQ */
#endif
#define FDT_TCP_ESTABLISHED 1

static int addr_str(const struct sockaddr_storage *ss, char *buf, size_t n, int *port)
{
	if (ss->ss_family == AF_INET) {
		const struct sockaddr_in *a = (const struct sockaddr_in *)ss;
		*port = ntohs(a->sin_port);
		return inet_ntop(AF_INET, &a->sin_addr, buf, (socklen_t)n) ? 0 : -1;
	}
	if (ss->ss_family == AF_INET6) {
		const struct sockaddr_in6 *a = (const struct sockaddr_in6 *)ss;
		*port = ntohs(a->sin6_port);
		if (!inet_ntop(AF_INET6, &a->sin6_addr, buf, (socklen_t)n))
			return -1;
		if (!strncmp(buf, "::ffff:", 7) && strchr(buf + 7, '.'))   /* IPv4-mapped */
			memmove(buf, buf + 7, strlen(buf + 7) + 1);
		return 0;
	}
	return -1;
}

static int same_addr(const char *a, const char *b)
{
	if (!strncmp(a, "::ffff:", 7) && strchr(a + 7, '.'))
		a += 7;
	if (!strncmp(b, "::ffff:", 7) && strchr(b + 7, '.'))
		b += 7;
	return !strcmp(a, b);
}

int fdt_sock_find(int local_port, const char *peer_addr, ino_t *ino)
{
	DIR *d = opendir("/proc/self/fd");
	if (!d)
		return -1;
	struct dirent *e;
	uint32_t best_age = UINT32_MAX;
	int best = -1;
	ino_t best_ino = 0;
	while ((e = readdir(d)) != NULL) {
		char *end;
		long v = strtol(e->d_name, &end, 10);
		if (*end || v <= 2 || v == dirfd(d))
			continue;
		int fd = (int)v;
		struct stat st;
		if (fstat(fd, &st) || !S_ISSOCK(st.st_mode))
			continue;
		struct sockaddr_storage ss, ps;
		socklen_t l = sizeof ss;
		char lb[64], pb[64];
		int lp, pp;
		if (getsockname(fd, (struct sockaddr *)&ss, &l) || addr_str(&ss, lb, sizeof lb, &lp) || lp != local_port)
			continue;
		l = sizeof ps;
		if (getpeername(fd, (struct sockaddr *)&ps, &l) || addr_str(&ps, pb, sizeof pb, &pp))
			continue;   /* listening socket */
		if (peer_addr && *peer_addr && !same_addr(pb, peer_addr))
			continue;
		struct tcp_info ti;
		memset(&ti, 0, sizeof ti);
		l = sizeof ti;
		if (getsockopt(fd, IPPROTO_TCP, TCP_INFO, &ti, &l) || ti.tcpi_state != FDT_TCP_ESTABLISHED)
			continue;
		if (best < 0 || ti.tcpi_last_data_recv < best_age) {
			best_age = ti.tcpi_last_data_recv;
			best = fd;
			best_ino = st.st_ino;
		}
	}
	closedir(d);
	if (best >= 0 && ino)
		*ino = best_ino;
	return best;
}

int fdt_sock_same(int fd, ino_t ino)
{
	struct stat st;
	if (fd < 0 || fstat(fd, &st) || !S_ISSOCK(st.st_mode) || st.st_ino != ino)
		return -1;
	return 0;
}

static int bytes_acked(int fd, uint64_t *acked)
{
	struct tcp_info ti;
	socklen_t l = sizeof ti;
	memset(&ti, 0, sizeof ti);
	if (getsockopt(fd, IPPROTO_TCP, TCP_INFO, &ti, &l) ||
	    l < offsetof(struct tcp_info, tcpi_bytes_acked) + sizeof ti.tcpi_bytes_acked)
		return -1;
	*acked = ti.tcpi_bytes_acked;
	return 0;
}

int fdt_sock_received(int fd, uint64_t *rx)
{
	struct tcp_info ti;
	socklen_t l = sizeof ti;
	memset(&ti, 0, sizeof ti);
	if (getsockopt(fd, IPPROTO_TCP, TCP_INFO, &ti, &l) ||
	    l < offsetof(struct tcp_info, tcpi_bytes_received) + sizeof ti.tcpi_bytes_received)
		return -1;
	*rx = ti.tcpi_bytes_received;
	return 0;
}

/* SIOCOUTQ and tcpi_bytes_acked come from two system calls, and an acknowledgment that the kernel
 * processes between them (softirq, another CPU) makes acked + outq overshoot the bytes written.
 * snd_una only grows, so the reads acked, outq, acked with the same value of acked are consistent. */
int fdt_sock_read(int fd, struct fdt_sockinfo *si)
{
	int v = 0;
	memset(si, 0, sizeof *si);
	for (int attempt = 0; attempt < 8; attempt++) {
		uint64_t a1 = 0, a2 = 0;
		int has = !bytes_acked(fd, &a1);
		if (ioctl(fd, SIOCOUTQ, &v))
			return -1;
		si->outq = v;
		if (ioctl(fd, SIOCOUTQNSD, &v))
			return -1;
		si->nsd = v;
		if (!has)
			return 0;
		if (!bytes_acked(fd, &a2) && a1 == a2) {
			si->acked = a2;
			si->has_acked = 1;
			return 0;
		}
	}
	return 0;   /* no consistent pair: SIOCOUTQ alone */
}
