#include <fleet_dt/queue.h>

int fdt_queue_init(fdt_queue_t *q, fdt_state_t *storage, size_t cap)
{
    if (q == NULL || storage == NULL || cap == 0) {
        return -1;
    }
    q->buf  = storage;
    q->cap  = cap;
    q->len  = 0;
    q->head = 0;
    return 0;
}

void fdt_queue_push(fdt_queue_t *q, const fdt_state_t *b)
{
    if (q == NULL || b == NULL || q->buf == NULL || q->cap == 0) {
        return;
    }

    if (q->len < q->cap) {
        q->buf[(q->head + q->len) % q->cap] = *b;
        q->len++;
    } else {
        /* Full: overwrite the oldest entry and slide the window forward. This
         * is what turns the capacity into the hard 48d ceiling of Section IV
         * rather than a soft target. */
        q->buf[q->head] = *b;
        q->head = (q->head + 1) % q->cap;
    }
}

size_t fdt_queue_len(const fdt_queue_t *q)
{
    return (q == NULL) ? 0 : q->len;
}

size_t fdt_queue_cap(const fdt_queue_t *q)
{
    return (q == NULL) ? 0 : q->cap;
}

const fdt_state_t *fdt_queue_at(const fdt_queue_t *q, size_t i)
{
    if (q == NULL || q->buf == NULL || i >= q->len) {
        return NULL;
    }
    return &q->buf[(q->head + i) % q->cap];
}

const fdt_state_t *fdt_queue_newest(const fdt_queue_t *q)
{
    if (q == NULL || q->len == 0) {
        return NULL;
    }
    return fdt_queue_at(q, q->len - 1);
}

size_t fdt_queue_bytes(size_t cap)
{
    return cap * sizeof(fdt_state_t);
}
