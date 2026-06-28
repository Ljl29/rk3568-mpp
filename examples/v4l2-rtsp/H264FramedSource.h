#ifndef H264_FRAMED_SOURCE_H
#define H264_FRAMED_SOURCE_H

#include <FramedSource.hh>
#include <stdint.h>
#include <stddef.h>

typedef struct TQueue TQueue;

/* MppPacket forward decl (from mpp_packet.h, opaque type) */
typedef void* MppPacket;

class H264FramedSource : public FramedSource {
public:
    static H264FramedSource* createNew(UsageEnvironment& env, TQueue* pkt_queue);

protected:
    H264FramedSource(UsageEnvironment& env, TQueue* pkt_queue);
    virtual ~H264FramedSource();

    virtual void doGetNextFrame();
    virtual void doStopGettingFrames();

private:
    static void deliverFrame0(void* clientData);
    void deliverFrame();
    void deliverOneNal();

    TQueue*       fPktQueue;
    MppPacket     fCurPkt;       /* current packet being split */
    const uint8_t* fCurPtr;      /* next byte to process */
    size_t        fRemaining;    /* bytes remaining in current packet */
    unsigned      fPollIntervalUs;
};

#ifdef __cplusplus
extern "C" {
#endif

void* h264_framed_source_new(void* env, void* pkt_queue);

#ifdef __cplusplus
}
#endif

#endif /* H264_FRAMED_SOURCE_H */
