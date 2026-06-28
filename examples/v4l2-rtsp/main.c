#define _GNU_SOURCE
#include "config.h"
#include "ts_queue.h"
#include "v4l2_capture.h"
#include "mpp_encoder.h"
#include "rtsp_server.h"

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
    TQueue       *frame_queue;
    TQueue       *pkt_queue;
    TQueue       *done_queue;
    int            width;
    int            height;
    int            drop;
    int            verbose;
    int64_t        capture_frames;
    int64_t        capture_drops;
    int64_t        encode_frames;
    int64_t        encode_errors;
} ThreadCtx;

/* ---- capture thread ---- */
static void* capture_thread(void *arg) {
    ThreadCtx *tc = (ThreadCtx*)arg;

    while (!g_stop) {
        int idx = v4l2_capture_get_frame(tc->cam);
        if (idx < 0) {
            if (!g_stop)
                LOG_ERR("V4L2 DQBUF failed, stopping");
            break;
        }

        void *done_item = NULL;
        while (tq_pop(tc->done_queue, &done_item, 0) == 0) {
            int done_idx = (int)(intptr_t)done_item;
            v4l2_capture_qbuf(tc->cam, done_idx);
        }

        FrameInfo *fi = (FrameInfo*)calloc(1, sizeof(FrameInfo));
        if (!fi) { LOG_ERR("calloc FrameInfo failed"); break; }
        fi->v4l2_idx    = idx;
        fi->v4l2_buf_ptr = v4l2_capture_get_data_ptr(tc->cam, idx);
        {
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            fi->ts_ms = (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
        }

        int push_ret = tq_push(tc->frame_queue, fi, tc->drop ? 100 : -1);
        if (push_ret != 0) {
            free(fi);
            v4l2_capture_qbuf(tc->cam, idx);
            tc->capture_drops++;
            continue;
        }
        tc->capture_frames++;
    }

    tq_flush(tc->frame_queue);
    return NULL;
}

/* ---- encode thread ---- */
static void* encode_thread(void *arg) {
    ThreadCtx *tc = (ThreadCtx*)arg;

    while (!g_stop) {
        FrameInfo *fi = NULL;
        if (tq_pop(tc->frame_queue, (void**)&fi, 500) != 0)
            continue;

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

        void *mpp_buf = v4l2_capture_get_mpp_buffer(tc->cam, fi->v4l2_idx);
        mpp_frame_set_buffer(frame, (MppBuffer)mpp_buf);

        MppPacket packet = NULL;
        ret = mpp_encoder_encode(tc->enc, frame, (void**)&packet);
        if (ret != 0 || !packet) {
            LOG_ERR("encode failed ret=%d", ret);
            tc->encode_errors++;
            mpp_frame_deinit(&frame);
            intptr_t idx_val = (intptr_t)fi->v4l2_idx;
            tq_push(tc->done_queue, (void*)idx_val, 100);
            free(fi);
            continue;
        }

        tc->encode_frames++;
        mpp_frame_deinit(&frame);

        int push_ret = tq_push(tc->pkt_queue, packet, tc->drop ? 100 : -1);
        if (push_ret != 0) {
            mpp_packet_deinit(&packet);
            tc->encode_errors++;
        }

        intptr_t idx_val = (intptr_t)fi->v4l2_idx;
        tq_push(tc->done_queue, (void*)idx_val, 100);

        free(fi);
    }

    tq_flush(tc->pkt_queue);
    return NULL;
}

/* ---- stats printer ---- */
static void print_stats(ThreadCtx *tc) {
    LOG("Stats: capture=%lld(+%lld drops) encode=%lld(+%lld errs) | fq=%d pq=%d",
        (long long)tc->capture_frames, (long long)tc->capture_drops,
        (long long)tc->encode_frames, (long long)tc->encode_errors,
        tq_count(tc->frame_queue), tq_count(tc->pkt_queue));
}

/* ---- Annex-B SPS/PPS extractor ---- */
static int extract_sps_pps(uint8_t *annexb_data, size_t annexb_len,
                           uint8_t **sps, int *sps_len,
                           uint8_t **pps, int *pps_len) {
    uint8_t *p = annexb_data;
    size_t remaining = annexb_len;
    *sps = *pps = NULL;
    *sps_len = *pps_len = 0;

    while (remaining > 3) {
        int sc_len = 0;
        if (p[0] == 0x00 && p[1] == 0x00 && p[2] == 0x00 && p[3] == 0x01)
            sc_len = 4;
        else if (p[0] == 0x00 && p[1] == 0x00 && p[2] == 0x01)
            sc_len = 3;
        else {
            p++; remaining--; continue;
        }
        p += sc_len; remaining -= (size_t)sc_len;

        uint8_t *next = p;
        size_t nal_len = 0;
        while (nal_len + 2 < remaining) {
            if (next[0] == 0x00 && next[1] == 0x00 &&
                (next[2] == 0x01 || (nal_len + 3 < remaining && next[2] == 0x00 && next[3] == 0x01)))
                break;
            next++; nal_len++;
        }
        if (nal_len == 0) { nal_len = remaining; }

        if (nal_len > 0) {
            int nal_type = p[0] & 0x1F;
            if (nal_type == 7 && *sps == NULL) {
                *sps = p; *sps_len = (int)nal_len;
            } else if (nal_type == 8 && *pps == NULL) {
                *pps = p; *pps_len = (int)nal_len;
            }
        }
        p = next;
        remaining -= nal_len;
    }
    return (*sps && *pps) ? 0 : -1;
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
        {"queue-len", required_argument, 0, 'q'},
        {"drop",      required_argument, 0, 'p'},
        {"verbose",   required_argument, 0, 'v'},
        {"help",      no_argument,       0, '?'},
        {0, 0, 0, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "d:w:h:f:b:r:g:q:p:v:", long_opts, NULL)) != -1) {
        switch (opt) {
        case 'd': strncpy(c->device, optarg, sizeof(c->device)-1); break;
        case 'w': c->width   = atoi(optarg); break;
        case 'h': c->height  = atoi(optarg); break;
        case 'f': c->fps     = atoi(optarg); break;
        case 'b': c->bitrate = atoi(optarg); break;
        case 'r': strncpy(c->rc_mode, optarg, sizeof(c->rc_mode)-1); break;
        case 'g': c->gop     = atoi(optarg); break;
        case 'q': c->queue_len = atoi(optarg); break;
        case 'p': c->drop    = atoi(optarg); break;
        case 'v': c->verbose = atoi(optarg); break;
        case '?':
        default:  usage(argv[0]); exit(0);
        }
    }
    return 0;
}

/* ---- main ---- */
int main(int argc, char **argv) {
    AppConfig cfg;
    if (parse_args(argc, argv, &cfg) != 0)
        return EXIT_FAILURE;

    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

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

    uint8_t *sps = NULL, *pps = NULL;
    int sps_len = 0, pps_len = 0;
    if (extract_sps_pps(hdr_data, hdr_len, &sps, &sps_len, &pps, &pps_len) != 0) {
        LOG_ERR("Failed to extract SPS/PPS from header");
        mpp_encoder_close(enc); v4l2_capture_close(cam); return EXIT_FAILURE;
    }
    LOG("Extracted SPS (%d bytes) and PPS (%d bytes)", sps_len, pps_len);

    TQueue frame_queue, pkt_queue, done_queue;
    if (tq_init(&frame_queue, cfg.queue_len) != 0 ||
        tq_init(&pkt_queue, PKT_QUEUE_LEN) != 0 ||
        tq_init(&done_queue, DEFAULT_V4L2_BUFS * 2) != 0) {
        LOG_ERR("Failed to init queues");
        goto CLEANUP;
    }

    ThreadCtx tc;
    memset(&tc, 0, sizeof(tc));
    tc.cam          = cam;
    tc.enc          = enc;
    tc.frame_queue  = &frame_queue;
    tc.pkt_queue    = &pkt_queue;
    tc.done_queue   = &done_queue;
    tc.width        = cfg.width;
    tc.height       = cfg.height;
    tc.drop         = cfg.drop;
    tc.verbose      = cfg.verbose;

    pthread_t t_cap, t_enc;
    pthread_create(&t_cap, NULL, capture_thread, &tc);
    pthread_create(&t_enc, NULL, encode_thread, &tc);

    int rtsp_ret = rtsp_server_start((void*)&pkt_queue, 554, "live",
                                     sps, sps_len, pps, pps_len, &g_stop);
    if (rtsp_ret != 0) {
        LOG_ERR("Failed to start RTSP server");
        g_stop = 1;
        tq_flush(&frame_queue);
        pthread_join(t_cap, NULL);
        pthread_join(t_enc, NULL);
        goto CLEANUP;
    }

    LOG("Pipeline running: %s -> %dx%d@%d -> H.264 %dbps -> rtsp://<ip>:554/live",
        cfg.device, cfg.width, cfg.height, cfg.fps, cfg.bitrate);

    while (!g_stop) {
        sleep(5);
        print_stats(&tc);
    }

    LOG("Shutting down...");
    rtsp_server_stop();
    tq_flush(&frame_queue);
    tq_flush(&pkt_queue);

    pthread_join(t_cap, NULL);
    pthread_join(t_enc, NULL);

    print_stats(&tc);
    LOG("Pipeline stopped.");

CLEANUP:
    tq_destroy(&frame_queue);
    tq_destroy(&pkt_queue);
    tq_destroy(&done_queue);
    mpp_encoder_close(enc);
    v4l2_capture_close(cam);
    return 0;
}
