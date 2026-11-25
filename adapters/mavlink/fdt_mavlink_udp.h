#ifndef FLEET_DT_MAVLINK_UDP_H
#define FLEET_DT_MAVLINK_UDP_H

#include "mavlink/fdt_mavlink_wire.h"

#include <stdint.h>

/**
 * @file fdt_mavlink_udp.h
 * @brief The Ardupilot link of Figure 3 over UDP.
 *
 * Ardupilot (and its SITL) streams MAVLink to a ground station over UDP,
 * port 14550 by convention. This endpoint binds a port, folds every
 * telemetry message it receives into an fdt_mav_ingest_t (I^t), and sends
 * A^t back to the address the autopilot spoke from, as
 * RC_CHANNELS_OVERRIDE, the only thing Figure 3 sends to Ardupilot.
 */

/** One autopilot link. Opaque fields; use the functions below. */
typedef struct {
    int              fd;
    fdt_mav_parser_t parser;
    uint8_t          seq;
    uint8_t          sysid;        /**< Ours, as a ground station. */
    uint8_t          compid;
    int              have_peer;
    uint8_t          peer_addr[128]; /**< struct sockaddr_storage. */
    unsigned         peer_len;
    uint8_t          target_sys;   /**< The autopilot's, learnt from it. */
    uint8_t          target_comp;
    uint64_t         messages;     /**< Telemetry messages folded in. */
} fdt_mav_udp_t;

/**
 * @brief Binds @p port on all addresses.
 * @return 0, or -1 when the socket cannot be opened or bound.
 */
int fdt_mav_udp_open(fdt_mav_udp_t *l, uint16_t port);

/**
 * @brief Waits up to @p timeout_ms, then drains every datagram waiting.
 * @return Telemetry messages folded into @p ing, or -1 on a socket error.
 */
int fdt_mav_udp_poll(fdt_mav_udp_t *l, fdt_mav_ingest_t *ing, int timeout_ms);

/**
 * @brief Sends A^t to the autopilot heard last.
 * @return 0, or -1 when no autopilot has spoken yet or the send fails.
 */
int fdt_mav_udp_send_act(fdt_mav_udp_t *l, const fdt_actuation_t *a);

/** @brief Sends a ground-station HEARTBEAT to the autopilot heard last. */
int fdt_mav_udp_heartbeat(fdt_mav_udp_t *l);

/** @brief Closes the socket. */
void fdt_mav_udp_close(fdt_mav_udp_t *l);

#endif /* FLEET_DT_MAVLINK_UDP_H */
