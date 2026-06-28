#include "ts_queue.h"
#include <stdlib.h>
#include <errno.h>
#include <sys/time.h>

int tq_init(TQueue *q, int capacity) {
    if (!q || capacity < 1) return -1;
    q->bufs = (void**)calloc((size_t)capacity, sizeof(void*));
    if (!q->bufs) return -1;
    q->capacity = capacity;
    q->head     = 0;
    q->tail     = 0;
    q->count    = 0;
    q->flushed  = 0;
    pthread_mutex_init(&q->mutex, NULL);
    pthread_cond_init(&q->not_empty, NULL);
    pthread_cond_init(&q->not_full, NULL);
    return 0;
}

static void timespec_add_ms(struct timespec *ts, int ms) {
    ts->tv_sec  += ms / 1000;
    ts->tv_nsec += (ms % 1000) * 1000000L;
    if (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec  += 1;
        ts->tv_nsec -= 1000000000L;
    }
}

int tq_push(TQueue *q, void *item, int timeout_ms) {
    pthread_mutex_lock(&q->mutex);

    while (q->count == q->capacity && !q->flushed) {
        if (timeout_ms < 0) {
            pthread_cond_wait(&q->not_full, &q->mutex);
        } else if (timeout_ms == 0) {
            pthread_mutex_unlock(&q->mutex);
            return -1;
        } else {
            struct timeval now;
            struct timespec ts;
            gettimeofday(&now, NULL);
            ts.tv_sec  = now.tv_sec;
            ts.tv_nsec = now.tv_usec * 1000;
            timespec_add_ms(&ts, timeout_ms);
            int rc = pthread_cond_timedwait(&q->not_full, &q->mutex, &ts);
            if (rc == ETIMEDOUT) {
                pthread_mutex_unlock(&q->mutex);
                return -1;
            }
        }
    }

    if (q->flushed) {
        pthread_mutex_unlock(&q->mutex);
        return -1;
    }

    q->bufs[q->head] = item;
    q->head = (q->head + 1) % q->capacity;
    q->count++;
    pthread_cond_signal(&q->not_empty);
    pthread_mutex_unlock(&q->mutex);
    return 0;
}

int tq_pop(TQueue *q, void **item, int timeout_ms) {
    pthread_mutex_lock(&q->mutex);

    while (q->count == 0 && !q->flushed) {
        if (timeout_ms < 0) {
            pthread_cond_wait(&q->not_empty, &q->mutex);
        } else if (timeout_ms == 0) {
            pthread_mutex_unlock(&q->mutex);
            return -1;
        } else {
            struct timeval now;
            struct timespec ts;
            gettimeofday(&now, NULL);
            ts.tv_sec  = now.tv_sec;
            ts.tv_nsec = now.tv_usec * 1000;
            timespec_add_ms(&ts, timeout_ms);
            int rc = pthread_cond_timedwait(&q->not_empty, &q->mutex, &ts);
            if (rc == ETIMEDOUT) {
                pthread_mutex_unlock(&q->mutex);
                return -1;
            }
        }
    }

    if (q->flushed && q->count == 0) {
        pthread_mutex_unlock(&q->mutex);
        return -1;
    }

    *item = q->bufs[q->tail];
    q->tail = (q->tail + 1) % q->capacity;
    q->count--;
    pthread_cond_signal(&q->not_full);
    pthread_mutex_unlock(&q->mutex);
    return 0;
}

int tq_count(TQueue *q) {
    pthread_mutex_lock(&q->mutex);
    int c = q->count;
    pthread_mutex_unlock(&q->mutex);
    return c;
}

void tq_flush(TQueue *q) {
    pthread_mutex_lock(&q->mutex);
    q->flushed = 1;
    pthread_cond_broadcast(&q->not_empty);
    pthread_cond_broadcast(&q->not_full);
    pthread_mutex_unlock(&q->mutex);
}

void tq_destroy(TQueue *q) {
    if (!q || !q->bufs) return;
    free(q->bufs);
    pthread_mutex_destroy(&q->mutex);
    pthread_cond_destroy(&q->not_empty);
    pthread_cond_destroy(&q->not_full);
    q->bufs = NULL;
}
