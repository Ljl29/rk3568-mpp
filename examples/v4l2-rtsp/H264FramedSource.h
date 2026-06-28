#ifndef H264_FRAMED_SOURCE_H
#define H264_FRAMED_SOURCE_H

#include <FramedSource.hh>

typedef struct TQueue TQueue;

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

    TQueue*  fPktQueue;
    unsigned fPollIntervalUs;
};

#ifdef __cplusplus
extern "C" {
#endif

void* h264_framed_source_new(void* env, void* pkt_queue);

#ifdef __cplusplus
}
#endif

#endif /* H264_FRAMED_SOURCE_H */
