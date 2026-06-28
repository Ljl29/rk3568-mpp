# V4L2 → MPP H.264 → RTSP/RTMP 推流 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a multi-threaded C program on RK3568 that captures 1080p30 NV12 from IMX415 via V4L2, encodes with MPP H.264 hardware, and pushes RTSP/RTMP via FFmpeg libavformat.

**Architecture:** Three threads (capture → encode → push) connected by two bounded pthread queues. V4L2 mmap buffers are imported as MppBuffer via dma-buf fd (EXT_DMA type), avoiding CPU copies. MPP sync encode() API used in dedicated thread. FFmpeg avformat handles RTSP (rtsp muxer) and RTMP (flv muxer) automatically from URL.

**Tech Stack:** C99, librockchip_mpp, libavformat/libavcodec/libavutil, libpthread, Linux V4L2

---

## File Map

| File | Creates | Responsibility |
|------|---------|----------------|
| `examples/v4l2-rtsp/config.h` | New | Shared types: `AppConfig`, `FrameInfo`, `PktInfo`, defaults, logging macros |
| `examples/v4l2-rtsp/ts_queue.h` | New | Bounded queue interface: `TQueue`, `tq_init/push/pop/flush/destroy` |
| `examples/v4l2-rtsp/ts_queue.c` | New | Queue impl with pthread mutex + cond, timeout, flush-on-stop |
| `examples/v4l2-rtsp/v4l2_capture.h` | New | V4L2 module interface: `V4L2Capture`, open/get_frame/close |
| `examples/v4l2-rtsp/v4l2_capture.c` | New | V4L2 open, S_FMT NV12, REQBUFS mmap, EXPBUF→MppBuffer, DQBUF/QBUF |
| `examples/v4l2-rtsp/mpp_encoder.h` | New | MPP encoder interface: `MppEncoder`, open/get_header/encode/close |
| `examples/v4l2-rtsp/mpp_encoder.c` | New | MPP create/init, prep+rc+codec cfg, GET_HDR_SYNC, encode loop |
| `examples/v4l2-rtsp/ff_pusher.h` | New | FFmpeg pusher interface: `FFPusher`, open/write/close |
| `examples/v4l2-rtsp/ff_pusher.c` | New | avformat init, stream+codecpar setup, write_header, write_frame, trailer |
| `examples/v4l2-rtsp/main.c` | New | getopt_long args, signal handler, thread launch, stats, cleanup |
| `examples/v4l2-rtsp/CMakeLists.txt` | New | Build config referencing MPP headers + libs |

---

### Task 1: Create directory and CMakeLists.txt

**Files:**
- Create: `examples/v4l2-rtsp/CMakeLists.txt`

- [ ] **Step 1: Create the project directory**

```bash
mkdir -p examples/v4l2-rtsp
```

- [ ] **Step 2: Write CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 2.8)
project(v4l2_mpp_rtsp C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -O2 -g")

# MPP public headers: rk_mpi.h, mpp_frame.h, mpp_buffer.h, etc.
include_directories(
    ${CMAKE_SOURCE_DIR}/../../inc
    ${CMAKE_SOURCE_DIR}
)

# FFmpeg headers — may need adjustment on target system
# Common paths: /usr/include/ffmpeg, /usr/local/include
# If these are not in default search path, add:
# include_directories(/path/to/ffmpeg/include)

add_executable(v4l2_mpp_rtsp
    main.c
    v4l2_capture.c
    mpp_encoder.c
    ff_pusher.c
    ts_queue.c
)

target_link_libraries(v4l2_mpp_rtsp
    rockchip_mpp
    avformat
    avcodec
    avutil
    pthread
)

install(TARGETS v4l2_mpp_rtsp RUNTIME DESTINATION bin)
```

- [ ] **Step 3: Verify CMake parses correctly (syntax check only, won't link)**

```bash
cd examples/v4l2-rtsp && mkdir -p build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../../build/linux/aarch64/arm.linux.cross.cmake 2>&1
# Expected: Configuring done. (will warn about missing source files — expected)
```

- [ ] **Step 4: Commit**

```bash
git add examples/v4l2-rtsp/CMakeLists.txt
git commit -m "feat: add v4l2-mpp-rtsp project skeleton with CMakeLists.txt"
```

---

### Task 2: config.h — shared types, defaults, logging

**Files:**
- Create: `examples/v4l2-rtsp/config.h`

- [ ] **Step 1: Write config.h**

```c
#ifndef V4L2_MPP_RTSP_CONFIG_H
#define V4L2_MPP_RTSP_CONFIG_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Default parameters */
#define DEFAULT_WIDTH       1920
#define DEFAULT_HEIGHT      1080
#define DEFAULT_FPS         30
#define DEFAULT_BITRATE     4000000
#define DEFAULT_GOP         60
#define DEFAULT_QUEUE_LEN   5
#define DEFAULT_V4L2_BUFS   4
#define PKT_QUEUE_LEN       12

/* Simple logging */
#define LOG_TAG "v4l2_mpp_rtsp"
#define LOG(fmt, ...) \
    fprintf(stderr, "[" LOG_TAG "] %s:%d " fmt "\n", __func__, __LINE__, ##__VA_ARGS__)
#define LOG_ERR(fmt, ...) \
    fprintf(stderr, "[" LOG_TAG " ERROR] %s:%d " fmt "\n", __func__, __LINE__, ##__VA_ARGS__)

/* App configuration — populated from CLI args */
typedef struct {
    char    device[128];       /* /dev/videoX */
    int     width;
    int     height;
    int     fps;
    int     bitrate;
    char    rc_mode[16];       /* "cbr" / "vbr" / "fixqp" */
    int     gop;
    int     profile;           /* 100=High, 77=Main, 66=Baseline */
    int     level;             /* 40 = Level 4.0 */
    char    url[256];          /* rtsp://... or rtmp://... */
    int     queue_len;
    int     drop;              /* 0=block on full, 1=drop oldest on full */
    int     verbose;           /* 0=silent, 1=per-frame stats */
} AppConfig;

/* Passed through frame_queue */
typedef struct {
    void     *v4l2_buf_ptr;    /* mmap start of this V4L2 buffer (for munmap ref) */
    int       v4l2_idx;        /* V4L2 buffer index (for QBUF) */
    int64_t   ts_ms;           /* capture timestamp */
} FrameInfo;

/* Default config */
static inline void config_set_defaults(AppConfig *c) {
    memset(c, 0, sizeof(*c));
    strcpy(c->device, "/dev/video0");
    c->width        = DEFAULT_WIDTH;
    c->height       = DEFAULT_HEIGHT;
    c->fps          = DEFAULT_FPS;
    c->bitrate      = DEFAULT_BITRATE;
    strcpy(c->rc_mode, "cbr");
    c->gop          = DEFAULT_GOP;
    c->profile      = 100;
    c->level        = 40;
    c->queue_len    = DEFAULT_QUEUE_LEN;
    c->drop         = 1;
    c->verbose      = 0;
}

#endif /* V4L2_MPP_RTSP_CONFIG_H */
```

- [ ] **Step 2: Commit**

```bash
git add examples/v4l2-rtsp/config.h
git commit -m "feat: add config.h with shared types and defaults"
```

---

### Task 3: ts_queue.h — thread-safe bounded queue interface

**Files:**
- Create: `examples/v4l2-rtsp/ts_queue.h`

- [ ] **Step 1: Write ts_queue.h**

```c
#ifndef TS_QUEUE_H
#define TS_QUEUE_H

#include <pthread.h>

typedef struct {
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
```

- [ ] **Step 2: Commit**

```bash
git add examples/v4l2-rtsp/ts_queue.h
git commit -m "feat: add ts_queue.h bounded queue interface"
```

---

### Task 4: ts_queue.c — queue implementation

**Files:**
- Create: `examples/v4l2-rtsp/ts_queue.c`

- [ ] **Step 1: Write ts_queue.c**

```c
#include "ts_queue.h"
#include <stdlib.h>
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
```

- [ ] **Step 2: Commit**

```bash
git add examples/v4l2-rtsp/ts_queue.c
git commit -m "feat: implement ts_queue.c bounded queue with pthread cond"
```

---

### Task 5: v4l2_capture.h — V4L2 capture module interface

**Files:**
- Create: `examples/v4l2-rtsp/v4l2_capture.h`

- [ ] **Step 1: Write v4l2_capture.h**

```c
#ifndef V4L2_CAPTURE_H
#define V4L2_CAPTURE_H

#include <linux/videodev2.h>

typedef struct V4L2Capture V4L2Capture;

/*
 * Open and configure V4L2 device.
 *  dev     : device path, e.g. "/dev/video0"
 *  width   : desired width
 *  height  : desired height
 *  fps     : desired frame rate (used in V4L2_CID or set by driver)
 *  buf_cnt : number of kernel buffers to request (typically 4)
 *  Returns NULL on failure (prints error to stderr).
 */
V4L2Capture* v4l2_capture_open(const char *dev, int width, int height,
                               int fps, int buf_cnt);

/*
 * Dequeue one frame buffer from V4L2.
 * Returns v4l2 buffer index on success (>=0), -1 on error.
 * The corresponding MppBuffer is available via v4l2_capture_get_mpp_buffer().
 * Caller MUST call v4l2_capture_qbuf(idx) when done with the buffer.
 */
int  v4l2_capture_get_frame(V4L2Capture *c);

/*
 * Return the MppBuffer associated with a dequeued V4L2 buffer index.
 * Returns NULL if index is invalid.
 */
void* v4l2_capture_get_mpp_buffer(V4L2Capture *c, int idx);

/*
 * Return the mmap'd data pointer for a dequeued V4L2 buffer.
 */
void* v4l2_capture_get_data_ptr(V4L2Capture *c, int idx);

/*
 * Return data size (bytesused) for a dequeued V4L2 buffer.
 */
size_t v4l2_capture_get_data_size(V4L2Capture *c, int idx);

/*
 * Queue a buffer back to V4L2 for reuse.
 * Call after encoding is done with the buffer.
 */
int  v4l2_capture_qbuf(V4L2Capture *c, int idx);

/*
 * Stop streaming, unmap buffers, close device. Safe to call with NULL.
 */
void v4l2_capture_close(V4L2Capture *c);

#endif /* V4L2_CAPTURE_H */
```

- [ ] **Step 2: Commit**

```bash
git add examples/v4l2-rtsp/v4l2_capture.h
git commit -m "feat: add v4l2_capture.h interface"
```

---

### Task 6: v4l2_capture.c — V4L2 capture implementation

**Files:**
- Create: `examples/v4l2-rtsp/v4l2_capture.c`

- [ ] **Step 1: Write v4l2_capture.c**

```c
#include "v4l2_capture.h"
#include "config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/select.h>

/*
 * MPP buffer API: mpp_buffer_import, mpp_buffer_put, etc.
 * We use EXT_DMA type to wrap V4L2 dma-buf fds.
 * This header is available from the MPP inc/ directory.
 */
#include "mpp_buffer.h"

/* Per-buffer state */
typedef struct {
    void        *start;       /* mmap'd userspace pointer */
    size_t       length;      /* buffer length */
    int          export_fd;   /* dma-buf fd from VIDIOC_EXPBUF */
    MppBuffer    mpp_buf;     /* MppBuffer wrapping the dma-buf */
} V4L2Buf;

struct V4L2Capture {
    int             fd;
    int             buf_cnt;
    enum v4l2_buf_type buf_type;
    V4L2Buf         bufs[10];    /* max 10 buffers */
};

/*
 * ioctl wrapper that retries on EINTR/EAGAIN
 */
static int xioctl(int fd, unsigned long request, void *arg) {
    int ret;
    for (;;) {
        ret = ioctl(fd, request, arg);
        if (ret == -1 && (errno == EINTR || errno == EAGAIN))
            continue;
        break;
    }
    return ret;
}

V4L2Capture* v4l2_capture_open(const char *dev, int width, int height,
                               int fps, int buf_cnt) {
    if (buf_cnt > 10) buf_cnt = 10;

    V4L2Capture *c = (V4L2Capture*)calloc(1, sizeof(V4L2Capture));
    if (!c) { LOG_ERR("calloc V4L2Capture failed"); return NULL; }

    c->fd = open(dev, O_RDWR, 0);
    if (c->fd < 0) {
        LOG_ERR("Cannot open %s: %s", dev, strerror(errno));
        goto FAIL;
    }

    /* Query capabilities */
    struct v4l2_capability cap = {0};
    if (xioctl(c->fd, VIDIOC_QUERYCAP, &cap) < 0) {
        LOG_ERR("VIDIOC_QUERYCAP: %s", strerror(errno));
        goto FAIL;
    }
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) &&
        !(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE)) {
        LOG_ERR("Device does not support video capture");
        goto FAIL;
    }
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        LOG_ERR("Device does not support streaming I/O");
        goto FAIL;
    }

    /* Determine buffer type */
    if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE)
        c->buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    else
        c->buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    /* Set format: NV12 */
    struct v4l2_format fmt = {0};
    fmt.type                = c->buf_type;
    fmt.fmt.pix.width       = (unsigned int)width;
    fmt.fmt.pix.height      = (unsigned int)height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
    fmt.fmt.pix.field       = V4L2_FIELD_NONE;

    if (xioctl(c->fd, VIDIOC_S_FMT, &fmt) < 0) {
        LOG_ERR("VIDIOC_S_FMT NV12 %dx%d: %s", width, height, strerror(errno));
        goto FAIL;
    }
    LOG("Set format: %dx%d NV12 (got %dx%d)",
        width, height, fmt.fmt.pix.width, fmt.fmt.pix.height);

    /* Set frame rate */
    struct v4l2_streamparm parm = {0};
    parm.type = c->buf_type;
    parm.parm.capture.timeperframe.numerator   = 1;
    parm.parm.capture.timeperframe.denominator = (unsigned int)fps;
    xioctl(c->fd, VIDIOC_S_PARM, &parm);   /* best-effort, ignore error */

    /* Request mmap buffers */
    struct v4l2_requestbuffers req = {0};
    req.count  = (unsigned int)buf_cnt;
    req.type   = c->buf_type;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(c->fd, VIDIOC_REQBUFS, &req) < 0) {
        LOG_ERR("VIDIOC_REQBUFS mmap: %s", strerror(errno));
        goto FAIL;
    }
    c->buf_cnt = (int)req.count;
    LOG("Requested %d buffers, got %d", buf_cnt, c->buf_cnt);

    /* mmap each buffer and export dma-buf */
    for (int i = 0; i < c->buf_cnt; i++) {
        struct v4l2_buffer buf = {0};
        buf.type   = c->buf_type;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = (unsigned int)i;

        if (xioctl(c->fd, VIDIOC_QUERYBUF, &buf) < 0) {
            LOG_ERR("VIDIOC_QUERYBUF %d: %s", i, strerror(errno));
            goto FAIL;
        }

        c->bufs[i].length = buf.length;
        c->bufs[i].start  = mmap(NULL, buf.length,
                                 PROT_READ | PROT_WRITE, MAP_SHARED,
                                 c->fd, buf.m.offset);
        if (c->bufs[i].start == MAP_FAILED) {
            LOG_ERR("mmap buffer %d failed: %s", i, strerror(errno));
            goto FAIL;
        }

        /* Export dma-buf fd and import as MppBuffer */
        struct v4l2_exportbuffer expbuf = {0};
        expbuf.type  = c->buf_type;
        expbuf.index = (unsigned int)i;
        expbuf.flags = O_CLOEXEC;
        if (xioctl(c->fd, VIDIOC_EXPBUF, &expbuf) < 0) {
            LOG_ERR("VIDIOC_EXPBUF %d: %s", i, strerror(errno));
            goto FAIL;
        }
        c->bufs[i].export_fd = expbuf.fd;

        MppBufferInfo info;
        memset(&info, 0, sizeof(info));
        info.type  = MPP_BUFFER_TYPE_EXT_DMA;
        info.fd    = expbuf.fd;
        info.size  = buf.length;
        mpp_buffer_import(&c->bufs[i].mpp_buf, &info);

        LOG("Buffer %d: mmap=%p len=%zu dma-fd=%d mpp_buf=%p",
            i, c->bufs[i].start, buf.length, expbuf.fd,
            (void*)c->bufs[i].mpp_buf);
    }

    /* Queue all buffers to driver */
    for (int i = 0; i < c->buf_cnt; i++) {
        struct v4l2_buffer buf = {0};
        buf.type   = c->buf_type;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = (unsigned int)i;
        if (xioctl(c->fd, VIDIOC_QBUF, &buf) < 0) {
            LOG_ERR("VIDIOC_QBUF %d: %s", i, strerror(errno));
            goto FAIL;
        }
    }

    /* Start streaming */
    enum v4l2_buf_type type = c->buf_type;
    if (xioctl(c->fd, VIDIOC_STREAMON, &type) < 0) {
        LOG_ERR("VIDIOC_STREAMON: %s", strerror(errno));
        goto FAIL;
    }
    LOG("V4L2 streaming started");

    /* Skip initial unstable frames */
    for (int i = 0; i < 10; i++) {
        struct v4l2_buffer buf = {0};
        buf.type   = c->buf_type;
        buf.memory = V4L2_MEMORY_MMAP;
        if (xioctl(c->fd, VIDIOC_DQBUF, &buf) < 0) break;
        struct v4l2_buffer qb = {0};
        qb.type   = c->buf_type;
        qb.memory = V4L2_MEMORY_MMAP;
        qb.index  = buf.index;
        xioctl(c->fd, VIDIOC_QBUF, &qb);
    }
    LOG("Skipped %d initial frames for AWB/AE stabilization", 10);

    return c;

FAIL:
    v4l2_capture_close(c);
    return NULL;
}

int v4l2_capture_get_frame(V4L2Capture *c) {
    if (!c) return -1;

    struct v4l2_buffer buf = {0};
    buf.type   = c->buf_type;
    buf.memory = V4L2_MEMORY_MMAP;

    if (xioctl(c->fd, VIDIOC_DQBUF, &buf) < 0) {
        if (errno != EINTR)
            LOG_ERR("VIDIOC_DQBUF: %s", strerror(errno));
        return -1;
    }
    return (int)buf.index;
}

void* v4l2_capture_get_mpp_buffer(V4L2Capture *c, int idx) {
    if (!c || idx < 0 || idx >= c->buf_cnt) return NULL;
    return c->bufs[idx].mpp_buf;
}

void* v4l2_capture_get_data_ptr(V4L2Capture *c, int idx) {
    if (!c || idx < 0 || idx >= c->buf_cnt) return NULL;
    return c->bufs[idx].start;
}

size_t v4l2_capture_get_data_size(V4L2Capture *c, int idx) {
    if (!c || idx < 0 || idx >= c->buf_cnt) return 0;
    return c->bufs[idx].length;
}

int v4l2_capture_qbuf(V4L2Capture *c, int idx) {
    if (!c || idx < 0 || idx >= c->buf_cnt) return -1;

    struct v4l2_buffer buf = {0};
    buf.type   = c->buf_type;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index  = (unsigned int)idx;

    if (xioctl(c->fd, VIDIOC_QBUF, &buf) < 0) {
        LOG_ERR("VIDIOC_QBUF %d: %s", idx, strerror(errno));
        return -1;
    }
    return 0;
}

void v4l2_capture_close(V4L2Capture *c) {
    if (!c) return;

    if (c->fd >= 0) {
        /* Stop streaming */
        enum v4l2_buf_type type = c->buf_type;
        xioctl(c->fd, VIDIOC_STREAMOFF, &type);

        /* Release MppBuffers and unmap */
        for (int i = 0; i < c->buf_cnt; i++) {
            if (c->bufs[i].mpp_buf) {
                mpp_buffer_put(c->bufs[i].mpp_buf);
                c->bufs[i].mpp_buf = NULL;
            }
            if (c->bufs[i].start && c->bufs[i].start != MAP_FAILED) {
                munmap(c->bufs[i].start, c->bufs[i].length);
                c->bufs[i].start = NULL;
            }
            if (c->bufs[i].export_fd > 0) {
                close(c->bufs[i].export_fd);
                c->bufs[i].export_fd = -1;
            }
        }
        close(c->fd);
        c->fd = -1;
    }
    free(c);
}
```

- [ ] **Step 2: Commit**

```bash
git add examples/v4l2-rtsp/v4l2_capture.c
git commit -m "feat: implement v4l2_capture.c with mmap + dma-buf MppBuffer wrapping"
```

---

### Task 7: mpp_encoder.h — MPP H.264 encoder interface

**Files:**
- Create: `examples/v4l2-rtsp/mpp_encoder.h`

- [ ] **Step 1: Write mpp_encoder.h**

```c
#ifndef MPP_ENCODER_H
#define MPP_ENCODER_H

#include <stddef.h>
#include <stdint.h>

typedef struct MppEncoder MppEncoder;

/*
 * Create and initialize MPP H.264 encoder.
 *  width, height: input frame dimensions
 *  fps:    frame rate
 *  bps:    target bitrate in bits per second
 *  rc_mode: "cbr", "vbr", or "fixqp"
 *  gop:    GOP length (I-frame interval)
 *  Returns NULL on failure.
 */
MppEncoder* mpp_encoder_open(int width, int height, int fps, int bps,
                             const char *rc_mode, int gop);

/*
 * Get H.264 SPS+PPS header (Annex-B format).
 *  data: output pointer to header data (owned by encoder, do not free)
 *  len:  output header length in bytes
 *  Returns 0 on success, -1 on error.
 *  The returned data pointer is valid until mpp_encoder_close().
 */
int  mpp_encoder_get_header(MppEncoder *enc, uint8_t **data, size_t *len);

/*
 * Encode one frame (sync call, blocks until hardware completes).
 *  frame:  input MppFrame (must have MppBuffer attached)
 *  packet: output MppPacket (caller must mpp_packet_deinit after use)
 *  Returns 0 on success, non-zero on error.
 */
int  mpp_encoder_encode(MppEncoder *enc, void *frame, void **packet);

/*
 * Close encoder and release all MPP resources. Safe to call with NULL.
 */
void mpp_encoder_close(MppEncoder *enc);

#endif /* MPP_ENCODER_H */
```

- [ ] **Step 2: Commit**

```bash
git add examples/v4l2-rtsp/mpp_encoder.h
git commit -m "feat: add mpp_encoder.h interface"
```

---

### Task 8: mpp_encoder.c — MPP H.264 encoder implementation

**Files:**
- Create: `examples/v4l2-rtsp/mpp_encoder.c`

- [ ] **Step 1: Write mpp_encoder.c**

```c
#include "mpp_encoder.h"
#include "config.h"

#include "rk_mpi.h"
#include "rk_venc_cfg.h"
#include "rk_venc_cmd.h"
#include "mpp_frame.h"
#include "mpp_packet.h"
#include "mpp_buffer.h"
#include "mpp_common.h"

#include <stdlib.h>
#include <string.h>

struct MppEncoder {
    MppCtx          ctx;
    MppApi         *mpi;
    MppEncCfg       cfg;
    MppEncRcMode    rc_mode;
    int             width;
    int             height;
    int             fps;
    int             bps;
    int             gop;
    /* Cached SPS+PPS header (obtained at init time) */
    uint8_t        *header_data;
    size_t          header_len;
    MppBuffer       header_buf;    /* underlying buffer for header_data */
};

MppEncoder* mpp_encoder_open(int width, int height, int fps, int bps,
                             const char *rc_mode, int gop) {
    MPP_RET ret;
    MppEncoder *enc = (MppEncoder*)calloc(1, sizeof(MppEncoder));
    if (!enc) { LOG_ERR("calloc MppEncoder failed"); return NULL; }
    enc->width  = width;
    enc->height = height;
    enc->fps    = fps;
    enc->bps    = bps;
    enc->gop    = gop;

    /* Parse rc_mode string */
    if (strcmp(rc_mode, "vbr") == 0)
        enc->rc_mode = MPP_ENC_RC_MODE_VBR;
    else if (strcmp(rc_mode, "fixqp") == 0)
        enc->rc_mode = MPP_ENC_RC_MODE_FIXQP;
    else
        enc->rc_mode = MPP_ENC_RC_MODE_CBR;

    /* Create MPP context */
    ret = mpp_create(&enc->ctx, &enc->mpi);
    if (ret != MPP_OK) { LOG_ERR("mpp_create failed ret=%d", ret); goto FAIL; }

    /* Initialize as encoder, H.264 */
    ret = mpp_init(enc->ctx, MPP_CTX_ENC, MPP_VIDEO_CodingAVC);
    if (ret != MPP_OK) { LOG_ERR("mpp_init failed ret=%d", ret); goto FAIL; }

    /* Create and populate encoder config */
    ret = mpp_enc_cfg_init(&enc->cfg);
    if (ret != MPP_OK) { LOG_ERR("mpp_enc_cfg_init failed ret=%d", ret); goto FAIL; }

    /* --- prep config --- */
    mpp_enc_cfg_set_s32(enc->cfg, "prep:width",       width);
    mpp_enc_cfg_set_s32(enc->cfg, "prep:height",      height);
    mpp_enc_cfg_set_s32(enc->cfg, "prep:hor_stride",  width);
    mpp_enc_cfg_set_s32(enc->cfg, "prep:ver_stride",  height);
    mpp_enc_cfg_set_s32(enc->cfg, "prep:format",      MPP_FMT_YUV420SP);

    /* --- rc config --- */
    mpp_enc_cfg_set_s32(enc->cfg, "rc:mode",          enc->rc_mode);
    mpp_enc_cfg_set_s32(enc->cfg, "rc:bps_target",    bps);
    mpp_enc_cfg_set_s32(enc->cfg, "rc:bps_max",       bps * 17 / 16);
    mpp_enc_cfg_set_s32(enc->cfg, "rc:bps_min",       enc->rc_mode == MPP_ENC_RC_MODE_CBR ? bps * 15 / 16 : bps / 16);
    mpp_enc_cfg_set_s32(enc->cfg, "rc:fps_in_flex",   0);
    mpp_enc_cfg_set_s32(enc->cfg, "rc:fps_in_num",    fps);
    mpp_enc_cfg_set_s32(enc->cfg, "rc:fps_in_denorm", 1);
    mpp_enc_cfg_set_s32(enc->cfg, "rc:fps_out_flex",  0);
    mpp_enc_cfg_set_s32(enc->cfg, "rc:fps_out_num",   fps);
    mpp_enc_cfg_set_s32(enc->cfg, "rc:fps_out_denorm",1);
    mpp_enc_cfg_set_s32(enc->cfg, "rc:gop",           gop);
    mpp_enc_cfg_set_u32(enc->cfg, "rc:drop_mode",     MPP_ENC_RC_DROP_FRM_DISABLED);

    /* --- codec config: H.264 High Profile, Level 4.0, CABAC --- */
    mpp_enc_cfg_set_s32(enc->cfg, "codec:type",       MPP_VIDEO_CodingAVC);
    mpp_enc_cfg_set_s32(enc->cfg, "h264:profile",     100);
    mpp_enc_cfg_set_s32(enc->cfg, "h264:level",       40);
    mpp_enc_cfg_set_s32(enc->cfg, "h264:cabac_en",    1);
    mpp_enc_cfg_set_s32(enc->cfg, "h264:cabac_idc",   0);
    mpp_enc_cfg_set_s32(enc->cfg, "h264:trans8x8",    1);
    mpp_enc_cfg_set_s32(enc->cfg, "h264:qp_init",     26);
    mpp_enc_cfg_set_s32(enc->cfg, "h264:qp_max",      51);
    mpp_enc_cfg_set_s32(enc->cfg, "h264:qp_min",      10);
    mpp_enc_cfg_set_s32(enc->cfg, "h264:qp_max_i",    46);
    mpp_enc_cfg_set_s32(enc->cfg, "h264:qp_min_i",    24);

    /* Submit config to MPP */
    ret = enc->mpi->control(enc->ctx, MPP_ENC_SET_CFG, enc->cfg);
    if (ret != MPP_OK) { LOG_ERR("MPP_ENC_SET_CFG failed ret=%d", ret); goto FAIL; }

    /* Set header mode: attach SPS/PPS before each IDR (needed for RTSP) */
    {
        RK_U32 header_mode = MPP_ENC_HEADER_MODE_EACH_IDR;
        ret = enc->mpi->control(enc->ctx, MPP_ENC_SET_HEADER_MODE, &header_mode);
        if (ret != MPP_OK) { LOG_ERR("SET_HEADER_MODE failed ret=%d", ret); goto FAIL; }
    }

    /* Get SPS+PPS header via GET_HDR_SYNC */
    {
        /* Allocate a temp buffer for the header */
        MppBufferInfo info;
        memset(&info, 0, sizeof(info));
        info.type = MPP_BUFFER_TYPE_NORMAL;
        info.size = 1024 * 1024;  /* 1MB should be more than enough */
        mpp_buffer_get(NULL, &enc->header_buf, info.size);

        MppPacket hdr_pkt = NULL;
        mpp_packet_init_with_buffer(&hdr_pkt, enc->header_buf);
        mpp_packet_set_length(hdr_pkt, 0);

        ret = enc->mpi->control(enc->ctx, MPP_ENC_GET_HDR_SYNC, hdr_pkt);
        if (ret != MPP_OK) { LOG_ERR("GET_HDR_SYNC failed ret=%d", ret); goto FAIL; }

        enc->header_data = (uint8_t*)mpp_packet_get_pos(hdr_pkt);
        enc->header_len  = mpp_packet_get_length(hdr_pkt);
        LOG("Got SPS+PPS header: %zu bytes", enc->header_len);

        /* header_pkt will be deinit'd; header_buf keeps the data alive */
        mpp_packet_deinit(&hdr_pkt);
    }

    LOG("MPP encoder initialized: %dx%d %dfps %dbps GOP=%d rc=%s",
        width, height, fps, bps, gop, rc_mode);
    return enc;

FAIL:
    mpp_encoder_close(enc);
    return NULL;
}

int mpp_encoder_get_header(MppEncoder *enc, uint8_t **data, size_t *len) {
    if (!enc || !data || !len) return -1;
    *data = enc->header_data;
    *len  = enc->header_len;
    return 0;
}

int mpp_encoder_encode(MppEncoder *enc, void *frame, void **packet) {
    if (!enc || !frame || !packet) return -1;

    /* Encode one frame synchronously */
    MPP_RET ret = enc->mpi->encode(enc->ctx, (MppFrame)frame, (MppPacket*)packet);
    if (ret != MPP_OK) {
        LOG_ERR("encode failed ret=%d", ret);
        return -1;
    }
    return 0;
}

void mpp_encoder_close(MppEncoder *enc) {
    if (!enc) return;
    if (enc->ctx && enc->mpi) {
        enc->mpi->reset(enc->ctx);
        mpp_destroy(enc->ctx);
        enc->ctx = NULL;
        enc->mpi = NULL;
    }
    if (enc->cfg) {
        mpp_enc_cfg_deinit(enc->cfg);
        enc->cfg = NULL;
    }
    if (enc->header_buf) {
        mpp_buffer_put(enc->header_buf);
        enc->header_buf  = NULL;
        enc->header_data = NULL;
    }
    free(enc);
}
```

- [ ] **Step 2: Commit**

```bash
git add examples/v4l2-rtsp/mpp_encoder.c
git commit -m "feat: implement mpp_encoder.c with H.264 High Profile CBR config"
```

---

### Task 9: ff_pusher.h — FFmpeg RTSP/RTMP pusher interface

**Files:**
- Create: `examples/v4l2-rtsp/ff_pusher.h`

- [ ] **Step 1: Write ff_pusher.h**

```c
#ifndef FF_PUSHER_H
#define FF_PUSHER_H

#include <stddef.h>
#include <stdint.h>

typedef struct FFPusher FFPusher;

/*
 * Open RTSP/RTMP push connection.
 *  url:       "rtsp://..." or "rtmp://..." — muxer auto-detected from URL
 *  width, height: video dimensions (for SDP / metadata)
 *  fps:       frame rate (for time_base calculation)
 *  extradata: H.264 SPS+PPS in Annex-B format (from mpp_encoder_get_header)
 *  extradata_len: length of extradata
 *  Returns NULL on failure (prints error to stderr).
 */
FFPusher* ff_pusher_open(const char *url, int width, int height, int fps,
                         uint8_t *extradata, size_t extradata_len);

/*
 * Push one encoded H.264 packet.
 *  packet: MppPacket containing H.264 Annex-B NAL unit(s)
 *  Returns 0 on success, -1 on error (connection lost, etc.)
 *  This call may block on network I/O.
 */
int  ff_pusher_write(FFPusher *p, void *packet);

/*
 * Write trailer and close connection. Safe to call with NULL.
 */
void ff_pusher_close(FFPusher *p);

#endif /* FF_PUSHER_H */
```

- [ ] **Step 2: Commit**

```bash
git add examples/v4l2-rtsp/ff_pusher.h
git commit -m "feat: add ff_pusher.h interface"
```

---

### Task 10: ff_pusher.c — FFmpeg pusher implementation

**Files:**
- Create: `examples/v4l2-rtsp/ff_pusher.c`

- [ ] **Step 1: Write ff_pusher.c**

```c
#include "ff_pusher.h"
#include "config.h"

#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>

#include "mpp_packet.h"

#include <stdlib.h>
#include <string.h>

struct FFPusher {
    AVFormatContext *fmt_ctx;
    AVStream        *video_st;
    int64_t          frame_count;
    int              fps;
    int              time_base_num;    /* numerator of stream time_base */
    int              time_base_den;    /* denominator of stream time_base */
};

/*
 * Convert H.264 Annex-B SPS+PPS to the format FFmpeg expects for extradata
 * (AVCC format: 4-byte size prefix instead of start codes).
 * For simplicity and since MPP outputs standard Annex-B, we pass it as-is
 * and let avformat handle conversion internally when needed.
 */
FFPusher* ff_pusher_open(const char *url, int width, int height, int fps,
                         uint8_t *extradata, size_t extradata_len) {
    if (!url || !extradata || extradata_len == 0) {
        LOG_ERR("ff_pusher_open: invalid arguments");
        return NULL;
    }

    FFPusher *p = (FFPusher*)calloc(1, sizeof(FFPusher));
    if (!p) { LOG_ERR("calloc FFPusher failed"); return NULL; }
    p->fps = fps;

    /* Initialize network (only once per process, but safe to call multiple times) */
    avformat_network_init();

    /* Allocate output context — auto-detect format from URL */
    int ret = avformat_alloc_output_context2(&p->fmt_ctx, NULL, NULL, url);
    if (ret < 0 || !p->fmt_ctx) {
        LOG_ERR("avformat_alloc_output_context2 failed: %s", av_err2str(ret));
        goto FAIL;
    }

    LOG("Output format: %s (%s)", p->fmt_ctx->oformat->name,
        p->fmt_ctx->oformat->long_name);

    /* Create video stream */
    p->video_st = avformat_new_stream(p->fmt_ctx, NULL);
    if (!p->video_st) {
        LOG_ERR("avformat_new_stream failed");
        goto FAIL;
    }
    p->video_st->id = p->fmt_ctx->nb_streams - 1;

    /* Configure codec parameters */
    AVCodecParameters *cpar = p->video_st->codecpar;
    cpar->codec_type = AVMEDIA_TYPE_VIDEO;
    cpar->codec_id   = AV_CODEC_ID_H264;
    cpar->width      = width;
    cpar->height     = height;

    /* Set extradata (SPS+PPS) — FFmpeg expects AVCC format for MP4/FLV muxers,
     * but we use the bitstream filter approach: we'll write Annex-B packets
     * and let the muxer handle conversion if needed.
     * For RTSP, the muxer expects Annex-B or converts automatically.
     * For FLV/RTMP, we may need to use h264_mp4toannexb bsf in reverse.
     * We set extradata and rely on the muxer's internal handling. */
    {
        /*
         * FFmpeg's RTSP muxer expects AVCC extradata for H.264.
         * We convert Annex-B SPS/PPS → AVCC here.
         * AVCC format: [1byte version][1byte profile][1byte compat][1byte level]
         *               [1byte NAL size length - 1 (0x03 for 4 bytes)]
         *               [1byte num SPS][2byte SPS len][SPS data]
         *               [1byte num PPS][2byte PPS len][PPS data]
         */
        uint8_t *avcc = (uint8_t*)av_mallocz(extradata_len + 256);
        if (!avcc) {
            LOG_ERR("av_mallocz extradata failed");
            goto FAIL;
        }
        int avcc_len = 0;

        /* Parse Annex-B to extract SPS and PPS NAL units */
        uint8_t *sps = NULL, *pps = NULL;
        int sps_len = 0, pps_len = 0;
        uint8_t *p_data = extradata;
        size_t remaining = extradata_len;

        while (remaining > 3) {
            /* Find start code 0x00 0x00 0x00 0x01 or 0x00 0x00 0x01 */
            int sc_len = 0;
            if (p_data[0] == 0x00 && p_data[1] == 0x00 && p_data[2] == 0x00 && p_data[3] == 0x01)
                sc_len = 4;
            else if (p_data[0] == 0x00 && p_data[1] == 0x00 && p_data[2] == 0x01)
                sc_len = 3;
            else {
                p_data++; remaining--; continue;
            }
            p_data += sc_len; remaining -= (size_t)sc_len;

            /* Find next start code or end */
            uint8_t *next = p_data;
            size_t nal_len = 0;
            while (nal_len < remaining - 2) {
                if (next[0] == 0x00 && next[1] == 0x00 && (next[2] == 0x01 || (next[2] == 0x00 && remaining > 3 && next[3] == 0x01)))
                    break;
                next++; nal_len++;
            }
            if (nal_len == 0) { nal_len = remaining; next = p_data + remaining; }

            /* Determine NAL type */
            if (nal_len > 0) {
                int nal_type = p_data[0] & 0x1F;
                if (nal_type == 7 && !sps) {      /* SPS */
                    sps = p_data; sps_len = (int)nal_len;
                } else if (nal_type == 8 && !pps) { /* PPS */
                    pps = p_data; pps_len = (int)nal_len;
                }
            }
            p_data = next;
        }

        if (!sps || !pps) {
            LOG_ERR("Could not find SPS/PPS in extradata");
            av_free(avcc);
            goto FAIL;
        }

        /* Build AVCC header */
        avcc[avcc_len++] = 0x01;              /* configurationVersion */
        avcc[avcc_len++] = sps[1];            /* AVCProfileIndication */
        avcc[avcc_len++] = sps[2];            /* profile_compatibility */
        avcc[avcc_len++] = sps[3];            /* AVCLevelIndication */
        avcc[avcc_len++] = 0xFF;              /* lengthSizeMinusOne (0xFF = 4 bytes) */

        avcc[avcc_len++] = 0xE1;              /* numOfSequenceParameterSets (0xE1 = 1 SPS) */
        avcc[avcc_len++] = (uint8_t)(sps_len >> 8);   /* SPS length high */
        avcc[avcc_len++] = (uint8_t)(sps_len & 0xFF); /* SPS length low */
        memcpy(avcc + avcc_len, sps, (size_t)sps_len); avcc_len += sps_len;

        avcc[avcc_len++] = 0x01;              /* numOfPictureParameterSets */
        avcc[avcc_len++] = (uint8_t)(pps_len >> 8);   /* PPS length high */
        avcc[avcc_len++] = (uint8_t)(pps_len & 0xFF); /* PPS length low */
        memcpy(avcc + avcc_len, pps, (size_t)pps_len); avcc_len += pps_len;

        cpar->extradata      = avcc;
        cpar->extradata_size = avcc_len;
        /* Note: avcc is owned by cpar, will be freed when fmt_ctx is freed */
    }

    /* Set time base: 90kHz is standard for H.264 streaming */
    p->time_base_num = 1;
    p->time_base_den = 90000;
    p->video_st->time_base = (AVRational){p->time_base_num, p->time_base_den};

    /* Open the output (network stream).
     * For RTSP/RTMP, avio_open is not used — avformat_write_header handles it. */
    ret = avformat_write_header(p->fmt_ctx, NULL);
    if (ret < 0) {
        LOG_ERR("avformat_write_header failed: %s (is the server running?)",
                av_err2str(ret));
        goto FAIL;
    }

    LOG("FFmpeg pusher connected: %s -> %s %dx%d %dfps",
        url, p->fmt_ctx->oformat->name, width, height, fps);
    return p;

FAIL:
    ff_pusher_close(p);
    return NULL;
}

int ff_pusher_write(FFPusher *p, void *packet) {
    if (!p || !packet) return -1;

    void *data_ptr  = mpp_packet_get_pos((MppPacket)packet);
    size_t data_len = mpp_packet_get_length((MppPacket)packet);

    if (!data_ptr || data_len == 0) return 0;  /* skip empty packets */

    AVPacket avpkt;
    av_init_packet(&avpkt);
    avpkt.data         = (uint8_t*)data_ptr;
    avpkt.size         = (int)data_len;
    avpkt.stream_index = p->video_st->index;

    /* Calculate PTS in 90kHz time base */
    int64_t pts = p->frame_count * (int64_t)p->time_base_den / p->fps;
    avpkt.pts = pts;
    avpkt.dts = pts;

    int ret = av_interleaved_write_frame(p->fmt_ctx, &avpkt);
    av_packet_unref(&avpkt);

    if (ret < 0) {
        LOG_ERR("av_interleaved_write_frame failed: %s", av_err2str(ret));
        return -1;
    }

    p->frame_count++;
    return 0;
}

void ff_pusher_close(FFPusher *p) {
    if (!p) return;
    if (p->fmt_ctx) {
        /* Write trailer only if header was written successfully */
        if (p->video_st) {
            av_write_trailer(p->fmt_ctx);
        }
        /* Close the output I/O context if it was opened separately */
        if (p->fmt_ctx->pb) {
            avio_closep(&p->fmt_ctx->pb);
        }
        avformat_free_context(p->fmt_ctx);
        p->fmt_ctx = NULL;
    }
    /* Note: avformat_network_deinit() not called here — may be called
     * by another instance. In a single-process app, safe to skip. */
    free(p);
}
```

- [ ] **Step 2: Commit**

```bash
git add examples/v4l2-rtsp/ff_pusher.c
git commit -m "feat: implement ff_pusher.c with Annex-B to AVCC conversion and avformat muxing"
```

---

### Task 11: main.c — main program with argument parsing, threads, signal handling

**Files:**
- Create: `examples/v4l2-rtsp/main.c`

- [ ] **Step 1: Write main.c**

```c
#define _GNU_SOURCE
#include "config.h"
#include "ts_queue.h"
#include "v4l2_capture.h"
#include "mpp_encoder.h"
#include "ff_pusher.h"

#include "mpp_frame.h"
#include "mpp_packet.h"
#include "mpp_common.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>

/* ---- global state ---- */
static volatile int g_stop = 0;

static void sig_handler(int sig) {
    (void)sig;
    g_stop = 1;
}

/* ---- thread arguments ---- */
typedef struct {
    V4L2Capture  *cam;
    MppEncoder   *enc;
    FFPusher     *pusher;
    TQueue       *frame_queue;
    TQueue       *pkt_queue;
    TQueue       *done_queue;      /* holds int* (v4l2 indices ready for QBUF) */
    int            width;
    int            height;
    int            drop;           /* 0=block, 1=drop */
    int            verbose;
    /* Per-thread counters */
    int64_t        capture_frames;
    int64_t        capture_drops;
    int64_t        encode_frames;
    int64_t        encode_errors;
    int64_t        push_frames;
    int64_t        push_errors;
    int64_t        push_bytes;
} ThreadCtx;

/* ---- capture thread ---- */
static void* capture_thread(void *arg) {
    ThreadCtx *tc = (ThreadCtx*)arg;

    while (!g_stop) {
        /* Dequeue a frame from V4L2 */
        int idx = v4l2_capture_get_frame(tc->cam);
        if (idx < 0) {
            if (!g_stop)
                LOG_ERR("V4L2 DQBUF failed, stopping");
            break;
        }

        /* Check for completed buffers to QBUF back */
        void *done_item = NULL;
        while (tq_pop(tc->done_queue, &done_item, 0) == 0) {
            int done_idx = (int)(intptr_t)done_item;
            v4l2_capture_qbuf(tc->cam, done_idx);
        }

        /* Allocate FrameInfo */
        FrameInfo *fi = (FrameInfo*)calloc(1, sizeof(FrameInfo));
        if (!fi) { LOG_ERR("calloc FrameInfo failed"); break; }
        fi->v4l2_idx    = idx;
        fi->v4l2_buf_ptr = v4l2_capture_get_data_ptr(tc->cam, idx);

        /* Push to frame queue */
        int push_ret = tq_push(tc->frame_queue, fi, tc->drop ? 100 : -1);
        if (push_ret != 0) {
            /* Queue full — drop this frame */
            free(fi);
            v4l2_capture_qbuf(tc->cam, idx);
            tc->capture_drops++;
            continue;
        }
        tc->capture_frames++;
    }

    /* Flush frame_queue so encode thread wakes up */
    tq_flush(tc->frame_queue);
    return NULL;
}

/* ---- encode thread ---- */
static void* encode_thread(void *arg) {
    ThreadCtx *tc = (ThreadCtx*)arg;

    while (!g_stop) {
        FrameInfo *fi = NULL;
        if (tq_pop(tc->frame_queue, (void**)&fi, 500) != 0)
            continue;  /* timeout or flushed */

        /* Create MppFrame from the V4L2 buffer */
        MppFrame frame = NULL;
        int ret = mpp_frame_init(&frame);
        if (ret != MPP_OK) {
            LOG_ERR("mpp_frame_init failed ret=%d", ret);
            if (fi) { v4l2_capture_qbuf(tc->cam, fi->v4l2_idx); free(fi); }
            continue;
        }

        mpp_frame_set_width(frame, tc->width);
        mpp_frame_set_height(frame, tc->height);
        mpp_frame_set_hor_stride(frame, tc->width);
        mpp_frame_set_ver_stride(frame, tc->height);
        mpp_frame_set_fmt(frame, MPP_FMT_YUV420SP);

        /* Attach the MppBuffer from V4L2 */
        void *mpp_buf = v4l2_capture_get_mpp_buffer(tc->cam, fi->v4l2_idx);
        mpp_frame_set_buffer(frame, (MppBuffer)mpp_buf);

        /* Encode */
        MppPacket packet = NULL;
        ret = mpp_encoder_encode(tc->enc, frame, (void**)&packet);
        if (ret != 0 || !packet) {
            LOG_ERR("encode failed ret=%d", ret);
            tc->encode_errors++;
            mpp_frame_deinit(&frame);
            /* Return buffer to V4L2 */
            intptr_t idx_val = (intptr_t)fi->v4l2_idx;
            tq_push(tc->done_queue, (void*)idx_val, 100);
            free(fi);
            continue;
        }

        tc->encode_frames++;
        mpp_frame_deinit(&frame);

        /* Push packet to pkt_queue */
        int push_ret = tq_push(tc->pkt_queue, packet, tc->drop ? 100 : -1);
        if (push_ret != 0) {
            /* Pkt queue full — drop this encoded packet */
            mpp_packet_deinit(&packet);
            tc->encode_errors++;
        }

        /* Signal V4L2 buffer can be reused */
        intptr_t idx_val = (intptr_t)fi->v4l2_idx;
        tq_push(tc->done_queue, (void*)idx_val, 100);

        free(fi);
    }

    /* Flush pkt_queue so push thread wakes up */
    tq_flush(tc->pkt_queue);
    return NULL;
}

/* ---- push thread ---- */
static void* push_thread(void *arg) {
    ThreadCtx *tc = (ThreadCtx*)arg;

    while (!g_stop) {
        MppPacket packet = NULL;
        if (tq_pop(tc->pkt_queue, (void**)&packet, 500) != 0)
            continue;

        /* Read size before deinit (avoid use-after-free) */
        size_t pkt_len = mpp_packet_get_length(packet);

        int ret = ff_pusher_write(tc->pusher, packet);
        mpp_packet_deinit(&packet);

        if (ret != 0) {
            tc->push_errors++;
            if (!g_stop) {
                LOG_ERR("Push failed, stopping");
                g_stop = 1;
                break;
            }
        }
        tc->push_frames++;
        tc->push_bytes += (int64_t)pkt_len;
    }
    return NULL;
}

/* ---- stats printer ---- */
static void print_stats(ThreadCtx *tc) {
    LOG("Stats: capture=%lld(+%lld drops) encode=%lld(+%lld errs) push=%lld(+%lld errs) | "
        "fq=%d pq=%d",
        (long long)tc->capture_frames, (long long)tc->capture_drops,
        (long long)tc->encode_frames, (long long)tc->encode_errors,
        (long long)tc->push_frames, (long long)tc->push_errors,
        tq_count(tc->frame_queue), tq_count(tc->pkt_queue));
}

/* ---- usage ---- */
static void usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s [OPTIONS]\n"
        "  --device     PATH   V4L2 device path (default /dev/video0)\n"
        "  --width      N      Capture width (default 1920)\n"
        "  --height     N      Capture height (default 1080)\n"
        "  --fps        N      Target frame rate (default 30)\n"
        "  --bitrate    N      Target bitrate in bps (default 4000000)\n"
        "  --rc-mode    MODE   Rate control: cbr/vbr/fixqp (default cbr)\n"
        "  --gop        N      GOP length (default 60)\n"
        "  --url        URL    RTSP/RTMP push URL (REQUIRED)\n"
        "  --queue-len  N      Frame queue depth (default 5)\n"
        "  --drop       0|1    Drop on full: 0=block 1=drop (default 1)\n"
        "  --verbose    0|1    Verbose logging (default 0)\n"
        "  --help              Print this help\n",
        prog);
}

static int parse_args(int argc, char **argv, AppConfig *c) {
    config_set_defaults(c);

    static struct option long_opts[] = {
        {"device",    required_argument, 0, 'd'},
        {"width",     required_argument, 0, 'w'},
        {"height",    required_argument, 0, 'h'},
        {"fps",       required_argument, 0, 'f'},
        {"bitrate",   required_argument, 0, 'b'},
        {"rc-mode",   required_argument, 0, 'r'},
        {"gop",       required_argument, 0, 'g'},
        {"url",       required_argument, 0, 'u'},
        {"queue-len", required_argument, 0, 'q'},
        {"drop",      required_argument, 0, 'p'},
        {"verbose",   required_argument, 0, 'v'},
        {"help",      no_argument,       0, '?'},
        {0, 0, 0, 0}
    };

    int opt;
    int url_set = 0;
    while ((opt = getopt_long(argc, argv, "d:w:h:f:b:r:g:u:q:p:v:", long_opts, NULL)) != -1) {
        switch (opt) {
        case 'd': strncpy(c->device, optarg, sizeof(c->device)-1); break;
        case 'w': c->width   = atoi(optarg); break;
        case 'h': c->height  = atoi(optarg); break;
        case 'f': c->fps     = atoi(optarg); break;
        case 'b': c->bitrate = atoi(optarg); break;
        case 'r': strncpy(c->rc_mode, optarg, sizeof(c->rc_mode)-1); break;
        case 'g': c->gop     = atoi(optarg); break;
        case 'u': strncpy(c->url, optarg, sizeof(c->url)-1); url_set = 1; break;
        case 'q': c->queue_len = atoi(optarg); break;
        case 'p': c->drop    = atoi(optarg); break;
        case 'v': c->verbose = atoi(optarg); break;
        case '?':
        default:  usage(argv[0]); exit(0);
        }
    }

    if (!url_set) {
        LOG_ERR("--url is required");
        usage(argv[0]);
        return -1;
    }
    return 0;
}

/* ---- main ---- */
int main(int argc, char **argv) {
    AppConfig cfg;
    if (parse_args(argc, argv, &cfg) != 0)
        return EXIT_FAILURE;

    /* Signal handling */
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    /* Initialize modules */
    V4L2Capture *cam = v4l2_capture_open(cfg.device, cfg.width, cfg.height,
                                         cfg.fps, DEFAULT_V4L2_BUFS);
    if (!cam) { LOG_ERR("Failed to open V4L2 device"); return EXIT_FAILURE; }

    MppEncoder *enc = mpp_encoder_open(cfg.width, cfg.height, cfg.fps,
                                       cfg.bitrate, cfg.rc_mode, cfg.gop);
    if (!enc) { LOG_ERR("Failed to init MPP encoder"); v4l2_capture_close(cam); return EXIT_FAILURE; }

    uint8_t *hdr_data = NULL;
    size_t   hdr_len  = 0;
    mpp_encoder_get_header(enc, &hdr_data, &hdr_len);
    if (!hdr_data || hdr_len == 0) {
        LOG_ERR("Failed to get SPS/PPS header");
        mpp_encoder_close(enc); v4l2_capture_close(cam); return EXIT_FAILURE;
    }

    FFPusher *pusher = ff_pusher_open(cfg.url, cfg.width, cfg.height, cfg.fps,
                                      hdr_data, hdr_len);
    if (!pusher) {
        LOG_ERR("Failed to open push connection to %s", cfg.url);
        mpp_encoder_close(enc); v4l2_capture_close(cam); return EXIT_FAILURE;
    }

    /* Initialize queues */
    TQueue frame_queue, pkt_queue, done_queue;
    if (tq_init(&frame_queue, cfg.queue_len) != 0 ||
        tq_init(&pkt_queue, PKT_QUEUE_LEN) != 0 ||
        tq_init(&done_queue, DEFAULT_V4L2_BUFS * 2) != 0) {
        LOG_ERR("Failed to init queues");
        goto CLEANUP;
    }

    /* Thread context */
    ThreadCtx tc;
    memset(&tc, 0, sizeof(tc));
    tc.cam          = cam;
    tc.enc          = enc;
    tc.pusher       = pusher;
    tc.frame_queue  = &frame_queue;
    tc.pkt_queue    = &pkt_queue;
    tc.done_queue   = &done_queue;
    tc.width        = cfg.width;
    tc.height       = cfg.height;
    tc.drop         = cfg.drop;
    tc.verbose      = cfg.verbose;

    /* Launch threads */
    pthread_t t_cap, t_enc, t_push;
    pthread_create(&t_cap,  NULL, capture_thread, &tc);
    pthread_create(&t_enc,  NULL, encode_thread,  &tc);
    pthread_create(&t_push, NULL, push_thread,    &tc);

    LOG("Pipeline running: %s -> %dx%d@%d -> H.264 %dbps -> %s",
        cfg.device, cfg.width, cfg.height, cfg.fps, cfg.bitrate, cfg.url);

    /* Main loop: print stats every 5 seconds until stop */
    while (!g_stop) {
        sleep(5);
        print_stats(&tc);
    }

    LOG("Shutting down...");
    tq_flush(&frame_queue);
    tq_flush(&pkt_queue);

    /* Join threads */
    pthread_join(t_cap,  NULL);
    pthread_join(t_enc,  NULL);
    pthread_join(t_push, NULL);

    print_stats(&tc);
    LOG("Pipeline stopped.");

CLEANUP:
    tq_destroy(&frame_queue);
    tq_destroy(&pkt_queue);
    tq_destroy(&done_queue);
    ff_pusher_close(pusher);
    mpp_encoder_close(enc);
    v4l2_capture_close(cam);
    return 0;
}
```

- [ ] **Step 2: Commit**

```bash
git add examples/v4l2-rtsp/main.c
git commit -m "feat: implement main.c with CLI, signal handling, 3-thread pipeline"
```

---

### Task 12: Build verification (dry-run) and final review

- [ ] **Step 1: Verify all files exist**

```bash
ls -la examples/v4l2-rtsp/
# Expected: CMakeLists.txt  config.h  ts_queue.h  ts_queue.c
#           v4l2_capture.h  v4l2_capture.c  mpp_encoder.h  mpp_encoder.c
#           ff_pusher.h  ff_pusher.c  main.c
```

- [ ] **Step 2: Verify CMake configuration (will fail on link, passes on config)**

```bash
cd examples/v4l2-rtsp/build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../../build/linux/aarch64/arm.linux.cross.cmake 2>&1
# Expected: Configuring done. Generate done. (or missing libs warning, not errors)
```

- [ ] **Step 3: Full build on target device**

```bash
# On the RK3568 development board:
cd /path/to/mpp-release/examples/v4l2-rtsp
mkdir -p build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../../build/linux/aarch64/arm.linux.cross.cmake
make -j$(nproc)
# Expected: v4l2_mpp_rtsp binary produced
```

- [ ] **Step 4: Smoke test**

```bash
# Verify help output (no device needed)
./v4l2_mpp_rtsp --help
# Expected: usage text printed

# Verify early failure when device doesn't exist
./v4l2_mpp_rtsp --device /dev/nonexistent --url rtsp://localhost:8554/test
# Expected: "Cannot open /dev/nonexistent" error, exit non-zero
```

- [ ] **Step 5: End-to-end test with camera**

```bash
# Start an RTSP server first (e.g., mediamtx / rtsp-simple-server)
# Then:
./v4l2_mpp_rtsp --device /dev/video0 --url rtsp://192.168.1.100:8554/live
# Expected: "Pipeline running" log, stats every 5s
# Verify with: ffplay rtsp://192.168.1.100:8554/live
```

- [ ] **Step 6: Final commit**

```bash
git add .
git commit -m "chore: verify build config and complete v4l2-mpp-rtsp implementation"
```

---

## Post-Implementation Verification Checklist

- [ ] V4L2 device opens and negotiates NV12 1920x1080 correctly
- [ ] MPP encoder returns valid SPS+PPS header (>0 bytes)
- [ ] First encoded frame includes IDR with SPS/PPS prefix
- [ ] FFmpeg connects to RTSP server and writes header
- [ ] Video stream plays back correctly in ffplay/VLC
- [ ] RTMP push also works with same binary (different URL)
- [ ] SIGINT (Ctrl+C) triggers clean shutdown in <2 seconds
- [ ] No memory leaks after 10-minute run (check with `top`/`meminfo`)
- [ ] Queue depth stays within bounds (not growing unbounded)
