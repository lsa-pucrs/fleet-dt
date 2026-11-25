#include <fleet_dt/relay.h>

#include <fleet_dt/codec.h>
#include <fleet_dt/envelope.h>

static void on_raw(const char *topic, const uint8_t *buf, size_t len,
                   void *user)
{
    (void)topic;
    (void)fdt_relay_offer((fdt_relay_t *)user, buf, len);
}

int fdt_relay_init(fdt_relay_t *rl, fdt_transport_t *tr,
                   fdt_regpub_t *stages, size_t n,
                   const double sensor_hz[FDT_SENSOR_COUNT])
{
    if (rl == NULL || tr == NULL || tr->subscribe == NULL ||
        stages == NULL || n == 0 || n > 0x10000u || sensor_hz == NULL) {
        return -1;
    }

    rl->tr       = tr;
    rl->stages   = stages;
    rl->n        = n;
    rl->received = 0;
    rl->rejected = 0;
    for (size_t v = 0; v < n; v++) {
        if (fdt_regpub_init(&stages[v], tr, (unsigned)v, sensor_hz, 0) != 0) {
            return -1;
        }
    }
    return tr->subscribe(tr->self, FDT_TOPIC_RAW_ALL, on_raw, rl);
}

int fdt_relay_offer(fdt_relay_t *rl, const uint8_t *buf, size_t len)
{
    if (rl == NULL) {
        return -1;
    }

    fdt_env_t      env;
    const uint8_t *payload = NULL;
    if (buf == NULL || fdt_env_decode(buf, len, &env, &payload) < 0) {
        rl->rejected++;
        return -1;
    }

    const int s = fdt_env_kind_sensor(env.kind);
    fdt_input_t sample = {0};
    if (s < 0 || (size_t)env.vessel >= rl->n || payload == NULL ||
        fdt_dec_sensor(payload, env.payload_len, (fdt_sensor_t)s,
                       &sample) < 0 ||
        fdt_regpub_offer(&rl->stages[env.vessel], (fdt_sensor_t)s,
                         &sample) < 0) {
        rl->rejected++;
        return -1;
    }
    rl->received++;
    return 0;
}

int fdt_relay_tick(fdt_relay_t *rl)
{
    if (rl == NULL) {
        return -1;
    }
    int sent = 0;
    for (size_t v = 0; v < rl->n; v++) {
        const int k = fdt_regpub_tick(&rl->stages[v]);
        if (k > 0) {
            sent += k;
        }
    }
    return sent;
}

uint64_t fdt_relay_received(const fdt_relay_t *rl)
{
    return (rl == NULL) ? 0 : rl->received;
}

uint64_t fdt_relay_rejected(const fdt_relay_t *rl)
{
    return (rl == NULL) ? 0 : rl->rejected;
}
