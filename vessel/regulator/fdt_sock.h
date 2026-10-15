/*
 * Fleet-DT v2: the socket of a data bridge in the vessel broker (Linux).
 *
 * The GS broker opens each data bridge to a dedicated listener port of the vessel broker, so the
 * socket of the data bridge is the established TCP socket of the broker process whose local port
 * is that listener port and whose peer address is the address of the bridge client
 * (mosquitto_client_address). The plugin API gives no access to the descriptor, so fdt_sock_find
 * scans /proc/self/fd; among several candidates (a reconnection that overlaps the old
 * connection) it takes the one that received data last (tcpi_last_data_recv), which at
 * MOSQ_EVT_CONNECT is the socket that just carried the CONNECT packet.
 */
#ifndef FDT_SOCK_H
#define FDT_SOCK_H

#include <stdint.h>
#include <sys/types.h>

struct fdt_sockinfo {
	int64_t nsd;        /* SIOCOUTQNSD: bytes in the send queue not yet sent */
	int64_t outq;       /* SIOCOUTQ: bytes in the send queue not yet acknowledged */
	uint64_t acked;     /* tcpi_bytes_acked (Linux >= 4.1) */
	int has_acked;
};

/* descriptor of the socket, or -1; *ino receives its inode */
int fdt_sock_find(int local_port, const char *peer_addr, ino_t *ino);
/* 0 when fd still is the socket with inode ino */
int fdt_sock_same(int fd, ino_t ino);
/* reads the queue counters; 0 on success */
int fdt_sock_read(int fd, struct fdt_sockinfo *si);

#endif
