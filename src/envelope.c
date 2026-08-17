#include <fleet_dt/envelope.h>

#include <string.h>

static void put_u32(uint8_t **p, uint32_t v)
{
    (*p)[0] = (uint8_t)(v & 0xFFu);
    (*p)[1] = (uint8_t)((v >> 8) & 0xFFu);
    (*p)[2] = (uint8_t)((v >> 16) & 0xFFu);
    (*p)[3] = (uint8_t)((v >> 24) & 0xFFu);
    *p += 4;
}

static void put_u16(uint8_t **p, uint16_t v)
{
    (*p)[0] = (uint8_t)(v & 0xFFu);
    (*p)[1] = (uint8_t)((v >> 8) & 0xFFu);
    *p += 2;
}

static uint32_t get_u32(const uint8_t **p)
{
    const uint32_t v = (uint32_t)(*p)[0] |
                       ((uint32_t)(*p)[1] << 8) |
                       ((uint32_t)(*p)[2] << 16) |
                       ((uint32_t)(*p)[3] << 24);
    *p += 4;
    return v;
}

static uint16_t get_u16(const uint8_t **p)
{
    const uint16_t v = (uint16_t)((uint16_t)(*p)[0] |
                                  (uint16_t)((uint16_t)(*p)[1] << 8));
    *p += 2;
    return v;
}

/** Whether a kind byte names a payload this format defines. */
static int kind_known(uint8_t kind)
{
    return kind == FDT_ENV_INPUT || kind == FDT_ENV_STATE ||
           kind == FDT_ENV_ACT   || kind == FDT_ENV_GOAL;
}

long fdt_env_encode(const fdt_env_t *env, const uint8_t *payload, size_t plen,
                    uint8_t *buf, size_t cap)
{
    if (env == NULL || buf == NULL) {
        return -1;
    }
    if (plen > 0 && payload == NULL) {
        return -1;
    }
    if ((size_t)env->payload_len != plen) {
        return -1;
    }
    if (!kind_known(env->kind)) {
        return -1;
    }
    if (cap < (size_t)FDT_ENV_HEADER_BYTES + plen) {
        return -1;
    }

    uint8_t *p = buf;
    put_u32(&p, FDT_ENV_MAGIC);
    put_u16(&p, env->vessel);
    *p++ = env->kind;
    *p++ = env->flags;
    put_u32(&p, env->seq);
    put_u32(&p, (uint32_t)plen);

    if (plen > 0) {
        memcpy(p, payload, plen);
    }
    return (long)((size_t)FDT_ENV_HEADER_BYTES + plen);
}

long fdt_env_decode(const uint8_t *buf, size_t len, fdt_env_t *env,
                    const uint8_t **payload_out)
{
    if (buf == NULL || env == NULL || len < (size_t)FDT_ENV_HEADER_BYTES) {
        return -1;
    }

    const uint8_t *p = buf;
    const uint32_t magic = get_u32(&p);
    if (magic != FDT_ENV_MAGIC) {
        return -1;
    }

    fdt_env_t out;
    out.magic  = magic;
    out.vessel = get_u16(&p);
    out.kind   = *p++;
    out.flags  = *p++;
    out.seq    = get_u32(&p);
    out.payload_len = get_u32(&p);

    if (!kind_known(out.kind)) {
        return -1;
    }
    if (len < (size_t)FDT_ENV_HEADER_BYTES + (size_t)out.payload_len) {
        return -1;
    }

    *env = out;
    if (payload_out != NULL) {
        *payload_out = (out.payload_len > 0) ? p : NULL;
    }
    return (long)((size_t)FDT_ENV_HEADER_BYTES + (size_t)out.payload_len);
}
