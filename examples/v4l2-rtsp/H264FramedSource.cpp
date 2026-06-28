#include "H264FramedSource.h"

extern "C" {
#include "ts_queue.h"
#include "mpp_packet.h"
#include "config.h"
#include <string.h>
#include <stdio.h>
}

static int s_nal_count = 0;  /* diag: total NALs delivered */

H264FramedSource* H264FramedSource::createNew(UsageEnvironment& env, TQueue* pkt_queue) {
    return new H264FramedSource(env, pkt_queue);
}

H264FramedSource::H264FramedSource(UsageEnvironment& env, TQueue* pkt_queue)
    : FramedSource(env)
    , fPktQueue(pkt_queue)
    , fCurPkt(NULL)
    , fCurPtr(NULL)
    , fRemaining(0)
    , fPollIntervalUs(10000)
{
}

H264FramedSource::~H264FramedSource() {
    if (fCurPkt) {
        mpp_packet_deinit((MppPacket*)&fCurPkt);
        fCurPkt = NULL;
    }
}

void H264FramedSource::doGetNextFrame() {
    deliverOneNal();
}

/*
 * Find next start code prefix (00 00 01 or 00 00 00 01) in data.
 * Returns length of start code prefix (3 or 4), or 0 if none found.
 * Sets *nal_len to the number of bytes before the next start code.
 */
static int find_start_code(const uint8_t *data, size_t size, size_t *nal_len) {
    for (size_t i = 0; i + 1 < size; i++) {
        if (data[i] == 0 && data[i+1] == 0) {
            if (i + 2 < size && data[i+2] == 1) {
                *nal_len = i;
                return 3;
            }
            if (i + 3 < size && data[i+2] == 0 && data[i+3] == 1) {
                *nal_len = i;
                return 4;
            }
        }
    }
    *nal_len = size;
    return 0; /* no more start codes */
}

void H264FramedSource::deliverOneNal() {
    /* If no remaining data, get next packet from queue */
    if (fRemaining == 0) {
        /* Deinit previous packet */
        if (fCurPkt) {
            mpp_packet_deinit((MppPacket*)&fCurPkt);
            fCurPkt = NULL;
            fCurPtr = NULL;
        }

        void* raw_pkt = NULL;
        int ret = tq_pop(fPktQueue, &raw_pkt, 0);
        if (ret != 0 || raw_pkt == NULL) {
            /* No data available — poll again later */
            if (!fPktQueue->flushed) {
                nextTask() = envir().taskScheduler().scheduleDelayedTask(
                    fPollIntervalUs, (TaskFunc*)deliverFrame0, this);
            } else {
                handleClosure();
            }
            return;
        }

        fCurPkt = (MppPacket)raw_pkt;
        fCurPtr = (const uint8_t*)mpp_packet_get_pos(fCurPkt);
        fRemaining = mpp_packet_get_length(fCurPkt);

        /* DIAG: log packet arrival (first 10, then every 30th) */
        {
            static int pkt_seq = 0;
            pkt_seq++;
            if (pkt_seq <= 10 || pkt_seq % 30 == 0)
                fprintf(stderr, "[v4l2_mpp_rtsp] H264FramedSource:%d DIAG pkt #%d arrived: %zuB\n",
                        __LINE__, pkt_seq, fRemaining);
        }
    }

    /* Skip any start code prefix at current position */
    size_t sc_len = 0;
    if (fRemaining >= 4 && fCurPtr[0]==0 && fCurPtr[1]==0 && fCurPtr[2]==0 && fCurPtr[3]==1)
        sc_len = 4;
    else if (fRemaining >= 3 && fCurPtr[0]==0 && fCurPtr[1]==0 && fCurPtr[2]==1)
        sc_len = 3;

    fCurPtr    += sc_len;
    fRemaining -= sc_len;

    if (fRemaining == 0) {
        /* Start code with no data after it — try again */
        deliverOneNal();
        return;
    }

    /* Find next start code boundary = end of this NAL unit */
    size_t nal_len = 0;
    int next_sc = find_start_code(fCurPtr, fRemaining, &nal_len);

    if (nal_len == 0) {
        /* Empty NAL — advance and try again */
        fRemaining = 0;
        deliverOneNal();
        return;
    }

    /* Deliver this NAL unit (without start code) */
    if (nal_len > fMaxSize) {
        fNumTruncatedBytes = (unsigned)(nal_len - fMaxSize);
        nal_len = fMaxSize;
    } else {
        fNumTruncatedBytes = 0;
    }
    memcpy(fTo, fCurPtr, nal_len);
    fFrameSize = (unsigned)nal_len;

    /* DIAG: log first 20 NALs and every 50th */
    s_nal_count++;
    if (s_nal_count <= 20 || s_nal_count % 50 == 0) {
        int nt = (nal_len > 0) ? (fCurPtr[0] & 0x1F) : -1;
        const char *name = (nt >= 1 && nt <= 8) ?
            ((const char*[]){"?","NON-IDR","?","?","?","IDR","SEI","SPS","PPS"})[nt] : "?";
        fprintf(stderr, "[v4l2_mpp_rtsp] H264FramedSource:%d DIAG deliver #%d NAL=%s size=%u fMax=%u\n",
                __LINE__, s_nal_count, name, (unsigned)nal_len, fMaxSize);
    }

    /* Advance past this NAL; leave start code for next call to skip */
    fCurPtr    += nal_len;
    fRemaining -= nal_len;

    if (fRemaining == 0 || next_sc == 0) {
        /* No more start codes = end of packet; will get next on next call */
        fRemaining = 0;
    }

    afterGetting(this);
}

void H264FramedSource::doStopGettingFrames() {
    FramedSource::doStopGettingFrames();
}

void H264FramedSource::deliverFrame0(void* clientData) {
    ((H264FramedSource*)clientData)->deliverFrame();
}

void H264FramedSource::deliverFrame() {
    doGetNextFrame();
}

extern "C" {

void* h264_framed_source_new(void* env, void* pkt_queue) {
    return H264FramedSource::createNew(
        *(UsageEnvironment*)env, (TQueue*)pkt_queue);
}

} // extern "C"
