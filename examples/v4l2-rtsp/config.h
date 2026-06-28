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
