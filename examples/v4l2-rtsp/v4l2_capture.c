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

#include "mpp_buffer.h"

#define FMT_NUM_PLANES  1

/* Per-buffer state */
typedef struct {
    void        *start;       /* mmap'd userspace pointer */
    size_t       length;      /* buffer length (plane 0 for mplane) */
    int          export_fd;   /* dma-buf fd from VIDIOC_EXPBUF */
    MppBuffer    mpp_buf;     /* MppBuffer wrapping the dma-buf */
} V4L2Buf;

struct V4L2Capture {
    int             fd;
    int             buf_cnt;
    int             is_mplane;     /* 1 if V4L2_CAP_VIDEO_CAPTURE_MPLANE */
    enum v4l2_buf_type buf_type;
    V4L2Buf         bufs[10];      /* max 10 buffers */
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

/*
 * Setup v4l2_buffer for multi/single plane.
 */
static void setup_v4l2_buffer(struct v4l2_buffer *buf, int is_mplane,
                              struct v4l2_plane *planes) {
    memset(buf, 0, sizeof(*buf));
    if (is_mplane) {
        buf->type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf->memory   = V4L2_MEMORY_MMAP;
        buf->m.planes = planes;
        buf->length   = FMT_NUM_PLANES;
    } else {
        buf->type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf->memory = V4L2_MEMORY_MMAP;
    }
}

static unsigned long get_mmap_offset(struct v4l2_buffer *buf, int is_mplane) {
    if (is_mplane)
        return buf->m.planes[0].m.mem_offset;
    else
        return buf->m.offset;
}

static unsigned int get_buffer_length(struct v4l2_buffer *buf, int is_mplane) {
    if (is_mplane)
        return buf->m.planes[0].length;
    else
        return buf->length;
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

    c->is_mplane = !!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE);
    if (c->is_mplane) {
        c->buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        LOG("Using multi-plane capture mode");
    } else {
        c->buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        LOG("Using single-plane capture mode");
    }

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
    xioctl(c->fd, VIDIOC_S_PARM, &parm);

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
        struct v4l2_plane planes[FMT_NUM_PLANES];
        struct v4l2_buffer buf;
        setup_v4l2_buffer(&buf, c->is_mplane, planes);
        buf.index = (unsigned int)i;

        if (xioctl(c->fd, VIDIOC_QUERYBUF, &buf) < 0) {
            LOG_ERR("VIDIOC_QUERYBUF %d: %s", i, strerror(errno));
            goto FAIL;
        }

        unsigned int buf_len = get_buffer_length(&buf, c->is_mplane);
        unsigned long offset = get_mmap_offset(&buf, c->is_mplane);

        c->bufs[i].length = buf_len;
        c->bufs[i].start  = mmap(NULL, buf_len,
                                 PROT_READ | PROT_WRITE, MAP_SHARED,
                                 c->fd, offset);
        if (c->bufs[i].start == MAP_FAILED) {
            LOG_ERR("mmap buffer %d failed: %s", i, strerror(errno));
            goto FAIL;
        }

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
        info.size  = buf_len;
        mpp_buffer_import(&c->bufs[i].mpp_buf, &info);

        LOG("Buffer %d: mmap=%p len=%u dma-fd=%d mpp_buf=%p",
            i, c->bufs[i].start, buf_len, expbuf.fd,
            (void*)c->bufs[i].mpp_buf);
    }

    /* Queue all buffers to driver */
    for (int i = 0; i < c->buf_cnt; i++) {
        struct v4l2_plane planes[FMT_NUM_PLANES];
        struct v4l2_buffer buf;
        setup_v4l2_buffer(&buf, c->is_mplane, planes);
        buf.index = (unsigned int)i;

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
        struct v4l2_plane planes[FMT_NUM_PLANES];
        struct v4l2_buffer buf;
        setup_v4l2_buffer(&buf, c->is_mplane, planes);

        if (xioctl(c->fd, VIDIOC_DQBUF, &buf) < 0) break;

        struct v4l2_buffer qb;
        setup_v4l2_buffer(&qb, c->is_mplane, planes);
        qb.index = buf.index;
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

    struct v4l2_plane planes[FMT_NUM_PLANES];
    struct v4l2_buffer buf;
    setup_v4l2_buffer(&buf, c->is_mplane, planes);

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

    struct v4l2_plane planes[FMT_NUM_PLANES];
    struct v4l2_buffer buf;
    setup_v4l2_buffer(&buf, c->is_mplane, planes);
    buf.index = (unsigned int)idx;

    if (xioctl(c->fd, VIDIOC_QBUF, &buf) < 0) {
        LOG_ERR("VIDIOC_QBUF %d: %s", idx, strerror(errno));
        return -1;
    }
    return 0;
}

void v4l2_capture_close(V4L2Capture *c) {
    if (!c) return;

    if (c->fd >= 0) {
        enum v4l2_buf_type type = c->buf_type;
        xioctl(c->fd, VIDIOC_STREAMOFF, &type);

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
