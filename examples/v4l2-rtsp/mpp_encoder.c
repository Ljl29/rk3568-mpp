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
    uint8_t        *header_data;
    size_t          header_len;
    MppBuffer       header_buf;
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

    if (strcmp(rc_mode, "vbr") == 0)
        enc->rc_mode = MPP_ENC_RC_MODE_VBR;
    else if (strcmp(rc_mode, "fixqp") == 0)
        enc->rc_mode = MPP_ENC_RC_MODE_FIXQP;
    else
        enc->rc_mode = MPP_ENC_RC_MODE_CBR;

    ret = mpp_create(&enc->ctx, &enc->mpi);
    if (ret != MPP_OK) { LOG_ERR("mpp_create failed ret=%d", ret); goto FAIL; }

    ret = mpp_init(enc->ctx, MPP_CTX_ENC, MPP_VIDEO_CodingAVC);
    if (ret != MPP_OK) { LOG_ERR("mpp_init failed ret=%d", ret); goto FAIL; }

    ret = mpp_enc_cfg_init(&enc->cfg);
    if (ret != MPP_OK) { LOG_ERR("mpp_enc_cfg_init failed ret=%d", ret); goto FAIL; }

    /* prep config */
    mpp_enc_cfg_set_s32(enc->cfg, "prep:width",       width);
    mpp_enc_cfg_set_s32(enc->cfg, "prep:height",      height);
    mpp_enc_cfg_set_s32(enc->cfg, "prep:hor_stride",  width);
    mpp_enc_cfg_set_s32(enc->cfg, "prep:ver_stride",  height);
    mpp_enc_cfg_set_s32(enc->cfg, "prep:format",      MPP_FMT_YUV420SP);

    /* rc config */
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

    /* codec config: H.264 High Profile, Level 4.0, CABAC */
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

    ret = enc->mpi->control(enc->ctx, MPP_ENC_SET_CFG, enc->cfg);
    if (ret != MPP_OK) { LOG_ERR("MPP_ENC_SET_CFG failed ret=%d", ret); goto FAIL; }

    {
        RK_U32 header_mode = MPP_ENC_HEADER_MODE_EACH_IDR;
        ret = enc->mpi->control(enc->ctx, MPP_ENC_SET_HEADER_MODE, &header_mode);
        if (ret != MPP_OK) { LOG_ERR("SET_HEADER_MODE failed ret=%d", ret); goto FAIL; }
    }

    /* Get SPS+PPS header */
    {
        MppBufferInfo info;
        memset(&info, 0, sizeof(info));
        info.type = MPP_BUFFER_TYPE_NORMAL;
        info.size = 1024 * 1024;
        mpp_buffer_get(NULL, &enc->header_buf, info.size);

        MppPacket hdr_pkt = NULL;
        mpp_packet_init_with_buffer(&hdr_pkt, enc->header_buf);
        mpp_packet_set_length(hdr_pkt, 0);

        ret = enc->mpi->control(enc->ctx, MPP_ENC_GET_HDR_SYNC, hdr_pkt);
        if (ret != MPP_OK) { LOG_ERR("GET_HDR_SYNC failed ret=%d", ret); goto FAIL; }

        enc->header_data = (uint8_t*)mpp_packet_get_pos(hdr_pkt);
        enc->header_len  = mpp_packet_get_length(hdr_pkt);
        LOG("Got SPS+PPS header: %zu bytes", enc->header_len);

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
