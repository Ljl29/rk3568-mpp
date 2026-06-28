#include "H264FramedSource.h"

extern "C" {
#include "ts_queue.h"
#include "mpp_packet.h"
#include "config.h"
}

H264FramedSource* H264FramedSource::createNew(UsageEnvironment& env, TQueue* pkt_queue) {
    return new H264FramedSource(env, pkt_queue);
}

H264FramedSource::H264FramedSource(UsageEnvironment& env, TQueue* pkt_queue)
    : FramedSource(env)
    , fPktQueue(pkt_queue)
    , fPollIntervalUs(10000)
{
}

H264FramedSource::~H264FramedSource() {
}

void H264FramedSource::doGetNextFrame() {
    void* raw_pkt = NULL;
    int ret = tq_pop(fPktQueue, &raw_pkt, 0);

    if (ret == 0 && raw_pkt != NULL) {
        MppPacket pkt = (MppPacket)raw_pkt;

        uint8_t* data = (uint8_t*)mpp_packet_get_pos(pkt);
        size_t   len  = mpp_packet_get_length(pkt);

        if (data && len > 0) {
            if (len > fMaxSize) len = fMaxSize;
            memcpy(fTo, data, len);
            fFrameSize = (unsigned)len;
            fNumTruncatedBytes = 0;
        } else {
            fFrameSize = 0;
        }

        mpp_packet_deinit(&pkt);

        if (fFrameSize > 0) {
            afterGetting(this);
            return;
        }
    }

    if (!fPktQueue->flushed) {
        nextTask() = envir().taskScheduler().scheduleDelayedTask(
            fPollIntervalUs, (TaskFunc*)deliverFrame0, this);
    } else {
        handleClosure();
    }
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
