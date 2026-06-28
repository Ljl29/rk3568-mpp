#ifndef TS_QUEUE_H
#define TS_QUEUE_H

#include <pthread.h>

typedef struct TQueue {
    void          **bufs;
    int             capacity;
    int             head;
    int             tail;
    int             count;
    pthread_mutex_t mutex;
    pthread_cond_t  not_empty;
    pthread_cond_t  not_full;
    int             flushed;
} TQueue;

/* Initialize queue with fixed capacity. Returns 0 on success, -1 on error. */
int  tq_init(TQueue *q, int capacity);

/*
 * Push item. If queue is full and timeout_ms > 0, blocks up to timeout_ms.
 * If timeout_ms == 0, returns immediately.
 * If timeout_ms < 0, blocks indefinitely.
 * Returns 0 on success, -1 on timeout/flushed.
 */
int  tq_push(TQueue *q, void *item, int timeout_ms);

/*
 * Pop item. Same timeout semantics as push.
 * Returns 0 on success, -1 on timeout/flushed.
 */
int  tq_pop(TQueue *q, void **item, int timeout_ms);

/* Return current count (non-blocking, approximate) */
int  tq_count(TQueue *q);

/* Wake all blocked threads. Subsequent push/pop return -1 immediately. */
void tq_flush(TQueue *q);

/* Free queue resources. Must call after all threads have exited. */
void tq_destroy(TQueue *q);

#endif /* TS_QUEUE_H */
