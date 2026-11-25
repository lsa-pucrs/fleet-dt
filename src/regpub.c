#include <fleet_dt/regpub.h>

#include <fleet_dt/envelope.h>

#include <stddef.h>

int fdt_regpub_init(fdt_regpub_t *rp, fdt_transport_t *tr, unsigned vessel,
                    const double sensor_hz[FDT_SENSOR_COUNT], int raw)
{
    if (rp == NULL || tr == NULL || tr->publish == NULL ||
        sensor_hz == NULL || vessel > 0xFFFFu) {
        return -1;
    }

    rp->tr       = tr;
    rp->vessel   = (uint16_t)vessel;
    rp->raw      = raw;
    rp->level    = FDT_TOPIC_LSDT;
    rp->held     = (fdt_input_t){0};
    rp->ticks    = 0;
    rp->messages = 0;
    rp->bytes    = 0;
    rp->refused  = 0;
    for (int s = 0; s < FDT_SENSOR_COUNT; s++) {
        if (fdt_reg_init(&rp->reg[s], sensor_hz[s]) != 0) {
            return -1;
        }
        rp->seq[s] = 0;
    }
    return 0;
}

/** Wraps the held sample of @p s in an envelope and publishes it. */
static int publish(fdt_regpub_t *rp, fdt_sensor_t s, uint32_t seq)
{
    uint8_t payload[FDT_WIRE_SENSOR_MAX_BYTES];
    const long plen = fdt_enc_sensor(&rp->held, s, payload, sizeof payload);
    if (plen < 0) {
        return -1;
    }

    const fdt_env_t env = { .magic = FDT_ENV_MAGIC,
                            .vessel = rp->vessel,
                            .kind = fdt_env_sensor_kind((int)s),
                            .flags = 0,
                            .seq = seq,
                            .payload_len = (uint32_t)plen };

    uint8_t frame[FDT_ENV_HEADER_BYTES + FDT_WIRE_SENSOR_MAX_BYTES];
    const long n = fdt_env_encode(&env, payload, (size_t)plen,
                                  frame, sizeof frame);
    if (n < 0) {
        return -1;
    }

    char topic[64];
    if (fdt_sensor_topic(topic, sizeof topic, rp->vessel, rp->level,
                         fdt_sensor_name(s)) != 0) {
        return -1;
    }

    if (rp->tr->publish(rp->tr->self, topic, frame, (size_t)n, 1) != 0) {
        rp->refused++;
        return -1;
    }
    rp->seq[s] = seq;
    rp->messages++;
    rp->bytes += (uint64_t)n;
    return 0;
}

int fdt_regpub_set_level(fdt_regpub_t *rp, const char *level)
{
    if (rp == NULL || level == NULL) {
        return -1;
    }
    rp->level = level;
    return 0;
}

int fdt_regpub_offer(fdt_regpub_t *rp, fdt_sensor_t s,
                     const fdt_input_t *sample)
{
    if (rp == NULL || sample == NULL || fdt_sensor_bytes(s) == 0) {
        return -1;
    }

    /* Copy only what this sensor measures into the held sample. */
    uint8_t buf[FDT_WIRE_SENSOR_MAX_BYTES];
    const long n = fdt_enc_sensor(sample, s, buf, sizeof buf);
    if (n < 0 || fdt_dec_sensor(buf, (size_t)n, s, &rp->held) < 0) {
        return -1;
    }

    fdt_reg_offer(&rp->reg[s]);
    if (!rp->raw) {
        return 0;
    }

    /* Raw: the regulator only counts; the sample leaves at once. */
    (void)fdt_reg_capture(&rp->reg[s]);
    return (publish(rp, s, rp->seq[s] + 1u) == 0) ? 1 : -1;
}

int fdt_regpub_tick(fdt_regpub_t *rp)
{
    if (rp == NULL) {
        return -1;
    }

    rp->ticks++;
    if (rp->raw) {
        return 0;
    }

    int sent = 0;
    for (int s = 0; s < FDT_SENSOR_COUNT; s++) {
        if (fdt_reg_capture(&rp->reg[s]) &&
            publish(rp, (fdt_sensor_t)s, rp->ticks) == 0) {
            sent++;
        }
    }
    return sent;
}

const fdt_reg_t *fdt_regpub_reg(const fdt_regpub_t *rp, fdt_sensor_t s)
{
    if (rp == NULL || fdt_sensor_bytes(s) == 0) {
        return NULL;
    }
    return &rp->reg[s];
}

uint64_t fdt_regpub_messages(const fdt_regpub_t *rp)
{
    return (rp == NULL) ? 0 : rp->messages;
}

uint64_t fdt_regpub_bytes(const fdt_regpub_t *rp)
{
    return (rp == NULL) ? 0 : rp->bytes;
}

uint64_t fdt_regpub_refused(const fdt_regpub_t *rp)
{
    return (rp == NULL) ? 0 : rp->refused;
}
