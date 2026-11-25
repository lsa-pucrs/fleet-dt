#define _POSIX_C_SOURCE 200809L

#include "mavlink/fdt_mavlink_udp.h"

#include <errno.h>
#include <netinet/in.h>
#include <poll.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

_Static_assert(sizeof(struct sockaddr_storage) <=
               sizeof(((fdt_mav_udp_t *)0)->peer_addr),
               "peer_addr too small for sockaddr_storage");

int fdt_mav_udp_open(fdt_mav_udp_t *l, uint16_t port)
{
    if (l == NULL) {
        return -1;
    }
    memset(l, 0, sizeof *l);
    l->sysid  = 255; /* the conventional ground-station system id */
    l->compid = 190; /* MAV_COMP_ID_MISSIONPLANNER */

    l->fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (l->fd < 0) {
        return -1;
    }
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family      = AF_INET;
    a.sin_port        = htons(port);
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(l->fd, (struct sockaddr *)&a, sizeof a) != 0) {
        close(l->fd);
        l->fd = -1;
        return -1;
    }
    return 0;
}

int fdt_mav_udp_poll(fdt_mav_udp_t *l, fdt_mav_ingest_t *ing, int timeout_ms)
{
    if (l == NULL || l->fd < 0 || ing == NULL) {
        return -1;
    }
    struct pollfd pfd = { .fd = l->fd, .events = POLLIN, .revents = 0 };
    const int pr = poll(&pfd, 1, (timeout_ms > 0) ? timeout_ms : 0);
    if (pr < 0) {
        return (errno == EINTR) ? 0 : -1;
    }

    int folded = 0;
    for (;;) {
        uint8_t dgram[2048];
        struct sockaddr_storage from;
        socklen_t flen = sizeof from;
        const ssize_t n = recvfrom(l->fd, dgram, sizeof dgram, MSG_DONTWAIT,
                                   (struct sockaddr *)&from, &flen);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break;
            }
            if (errno == EINTR) {
                continue;
            }
            return -1;
        }
        for (ssize_t i = 0; i < n; i++) {
            fdt_mav_msg_t m;
            if (!fdt_mav_parse(&l->parser, dgram[i], &m)) {
                continue;
            }
            /* The autopilot is whoever sends its HEARTBEAT or telemetry. */
            memcpy(l->peer_addr, &from, flen);
            l->peer_len    = (unsigned)flen;
            l->have_peer   = 1;
            l->target_sys  = m.sysid;
            l->target_comp = m.compid;
            if (fdt_mav_dispatch(ing, &m)) {
                folded++;
                l->messages++;
            }
        }
    }
    return folded;
}

static int send_frame(fdt_mav_udp_t *l, const uint8_t *b, long n)
{
    if (n < 0) {
        return -1;
    }
    const ssize_t w = sendto(l->fd, b, (size_t)n, 0,
                             (const struct sockaddr *)l->peer_addr,
                             (socklen_t)l->peer_len);
    l->seq++;
    return (w == n) ? 0 : -1;
}

int fdt_mav_udp_send_act(fdt_mav_udp_t *l, const fdt_actuation_t *a)
{
    if (l == NULL || l->fd < 0 || a == NULL || !l->have_peer) {
        return -1;
    }
    uint8_t b[FDT_MAV_MAX_FRAME];
    return send_frame(l, b, fdt_mav_enc_rc_override(b, sizeof b, l->seq,
                                                    l->sysid, l->compid,
                                                    l->target_sys,
                                                    l->target_comp, a));
}

int fdt_mav_udp_heartbeat(fdt_mav_udp_t *l)
{
    if (l == NULL || l->fd < 0 || !l->have_peer) {
        return -1;
    }
    uint8_t b[FDT_MAV_MAX_FRAME];
    return send_frame(l, b, fdt_mav_enc_heartbeat(b, sizeof b, l->seq,
                                                  l->sysid, l->compid));
}

void fdt_mav_udp_close(fdt_mav_udp_t *l)
{
    if (l != NULL && l->fd >= 0) {
        close(l->fd);
        l->fd = -1;
    }
}
