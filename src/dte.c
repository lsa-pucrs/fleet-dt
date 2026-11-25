#define _POSIX_C_SOURCE 200809L

#include <fleet_dt/dte.h>

#include <pthread.h>
#include <stddef.h>

typedef struct sync_s sync_t;

/** One worker thread: which simulation it runs. */
typedef struct {
    fdt_dte_t *dte;
    size_t     index;
    pthread_t  thread;
} worker_t;

/** What lives in fdt_dte_t.sync. */
struct sync_s {
    int               stopping;
    size_t            current;
    pthread_barrier_t go;
    pthread_barrier_t done;
    worker_t          workers[FDT_DTE_MAX_PARALLEL];
};

_Static_assert(sizeof(sync_t) <= FDT_DTE_SYNC_BYTES,
               "FDT_DTE_SYNC_BYTES too small for this platform's pthreads");
_Static_assert(_Alignof(sync_t) <= 16, "fdt_dte_t.sync under-aligned");

static sync_t *sync_of(fdt_dte_t *dte)
{
    return (sync_t *)(void *)dte->sync;
}

int fdt_dte_init(fdt_dte_t *dte, fdt_sim_slot_t *slots, size_t cap)
{
    if (dte == NULL || slots == NULL || cap == 0) {
        return -1;
    }
    dte->slots = slots;
    dte->cap   = cap;
    dte->n     = 0;
    dte->tick  = 0;
    dte->parallel = 0;
    return 0;
}

int fdt_dte_register(fdt_dte_t *dte, const char *name, fdt_sim_fn fn,
                     void *user)
{
    if (dte == NULL || dte->slots == NULL || name == NULL || fn == NULL) {
        return -1;
    }
    if (dte->n >= dte->cap || dte->parallel) {
        return -1;
    }
    dte->slots[dte->n].name = name;
    dte->slots[dte->n].fn   = fn;
    dte->slots[dte->n].user = user;
    dte->n++;
    return 0;
}

size_t fdt_dte_tick(fdt_dte_t *dte)
{
    if (dte == NULL || dte->slots == NULL) {
        return (size_t)-1;
    }

    const size_t t = dte->tick;

    if (dte->parallel) {
        sync_t *sy = sync_of(dte);
        sy->current = t;
        pthread_barrier_wait(&sy->go);   /* release every simulation */
        pthread_barrier_wait(&sy->done); /* the tick edge: all finished */
    } else {
        for (size_t i = 0; i < dte->n; i++) {
            dte->slots[i].fn(t, dte->slots[i].user);
        }
    }

    dte->tick++;
    return t;
}

static void *worker_main(void *arg)
{
    worker_t *w = (worker_t *)arg;
    fdt_dte_t *dte = w->dte;
    sync_t *sy = sync_of(dte);
    for (;;) {
        pthread_barrier_wait(&sy->go);
        if (sy->stopping) {
            break;
        }
        const fdt_sim_slot_t *s = &dte->slots[w->index];
        s->fn(sy->current, s->user);
        pthread_barrier_wait(&sy->done);
    }
    return NULL;
}

int fdt_dte_start_parallel(fdt_dte_t *dte)
{
    if (dte == NULL || dte->slots == NULL || dte->parallel || dte->n == 0 ||
        dte->n > FDT_DTE_MAX_PARALLEL) {
        return -1;
    }
    sync_t *sy = sync_of(dte);
    const unsigned parties = (unsigned)dte->n + 1u;
    if (pthread_barrier_init(&sy->go, NULL, parties) != 0) {
        return -1;
    }
    if (pthread_barrier_init(&sy->done, NULL, parties) != 0) {
        pthread_barrier_destroy(&sy->go);
        return -1;
    }
    sy->stopping = 0;
    sy->current  = 0;

    for (size_t i = 0; i < dte->n; i++) {
        sy->workers[i].dte   = dte;
        sy->workers[i].index = i;
        if (pthread_create(&sy->workers[i].thread, NULL, worker_main,
                           &sy->workers[i]) != 0) {
            /* A partial set cannot pass an n+1 barrier: cancel the threads
             * already waiting on it, then give up. */
            for (size_t k = 0; k < i; k++) {
                pthread_cancel(sy->workers[k].thread);
                pthread_join(sy->workers[k].thread, NULL);
            }
            pthread_barrier_destroy(&sy->go);
            pthread_barrier_destroy(&sy->done);
            return -1;
        }
    }
    dte->parallel = 1;
    return 0;
}

int fdt_dte_stop_parallel(fdt_dte_t *dte)
{
    if (dte == NULL || !dte->parallel) {
        return -1;
    }
    sync_t *sy = sync_of(dte);
    sy->stopping = 1;
    pthread_barrier_wait(&sy->go);
    for (size_t i = 0; i < dte->n; i++) {
        pthread_join(sy->workers[i].thread, NULL);
    }
    pthread_barrier_destroy(&sy->go);
    pthread_barrier_destroy(&sy->done);
    dte->parallel = 0;
    return 0;
}

int fdt_dte_is_parallel(const fdt_dte_t *dte)
{
    return (dte != NULL) ? dte->parallel : 0;
}

size_t fdt_dte_count(const fdt_dte_t *dte)
{
    return (dte == NULL) ? 0 : dte->n;
}

size_t fdt_dte_ticks(const fdt_dte_t *dte)
{
    return (dte == NULL) ? 0 : dte->tick;
}

const char *fdt_dte_name(const fdt_dte_t *dte, size_t i)
{
    if (dte == NULL || dte->slots == NULL || i >= dte->n) {
        return NULL;
    }
    return dte->slots[i].name;
}
