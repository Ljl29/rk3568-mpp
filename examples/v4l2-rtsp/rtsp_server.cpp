#include "rtsp_server.h"
#include "H264FramedSource.h"

#include <liveMedia.hh>
#include <BasicUsageEnvironment.hh>
#include <GroupsockHelper.hh>

extern "C" {
#include "ts_queue.h"
#include "config.h"
}

#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <atomic>

static std::atomic<char> g_srv_stop{0};
static pthread_t         g_event_thread;

/* ---- minimal base64 for sprop-parameter-sets ---- */
static const char b64_tbl[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static void base64_encode_buf(const uint8_t *in, unsigned in_len,
                               char *out, unsigned out_cap) {
    unsigned i = 0, j = 0;
    while (i < in_len && j < out_cap - 4) {
        uint32_t a = in[i++];
        uint32_t b = (i < in_len) ? in[i++] : 0;
        uint32_t c = (i < in_len) ? in[i++] : 0;
        uint32_t triple = (a << 16) | (b << 8) | c;
        out[j++] = b64_tbl[(triple >> 18) & 0x3F];
        out[j++] = b64_tbl[(triple >> 12) & 0x3F];
        out[j++] = b64_tbl[(triple >>  6) & 0x3F];
        out[j++] = b64_tbl[(triple >>  0) & 0x3F];
    }
    unsigned mod = in_len % 3;
    if (mod == 1) { j -= 2; if (j < out_cap) out[j++] = '='; if (j < out_cap) out[j++] = '='; }
    else if (mod == 2) { j -= 1; if (j < out_cap) out[j++] = '='; }
    out[j] = '\0';
}

/* Build "a=fmtp:<pt> sprop-parameter-sets=<b64sps>,<b64pps>\r\n" */
static char* build_fmtp_line(unsigned pt,
                              const uint8_t *sps, unsigned sps_len,
                              const uint8_t *pps, unsigned pps_len) {
    char b64_sps[512], b64_pps[256];
    base64_encode_buf(sps, sps_len, b64_sps, sizeof(b64_sps));
    base64_encode_buf(pps, pps_len, b64_pps, sizeof(b64_pps));

    char tmp[1024];
    snprintf(tmp, sizeof(tmp),
             "a=fmtp:%u sprop-parameter-sets=%s,%s\r\n",
             pt, b64_sps, b64_pps);
    return strdup(tmp);
}

// Custom subsession: creates H264FramedSource+framer+sink for each client.
// Provides out-of-band SPS/PPS both in SDP (sprop-parameter-sets) and
// in the RTP stream (via setVPSandSPSandPPS).
class PktQueueServerMediaSubsession : public OnDemandServerMediaSubsession {
public:
    static PktQueueServerMediaSubsession* createNew(UsageEnvironment& env,
            TQueue* pkt_queue, u_int8_t rtpPayloadType,
            u_int8_t* sps, unsigned spsSize, u_int8_t* pps, unsigned ppsSize) {
        return new PktQueueServerMediaSubsession(env, pkt_queue, rtpPayloadType,
                                                 sps, spsSize, pps, ppsSize);
    }

protected:
    PktQueueServerMediaSubsession(UsageEnvironment& env, TQueue* pkt_queue,
                                  u_int8_t rtpPayloadType,
                                  u_int8_t* sps, unsigned spsSize,
                                  u_int8_t* pps, unsigned ppsSize)
        : OnDemandServerMediaSubsession(env, True /*reuseFirstSource*/),
          fPktQueue(pkt_queue), fRtpPayloadType(rtpPayloadType),
          fSPS(sps), fSPSSize(spsSize), fPPS(pps), fPPSSize(ppsSize),
          fFmtpLine(NULL) {
        fFmtpLine = build_fmtp_line((unsigned)rtpPayloadType,
                                     sps, spsSize, pps, ppsSize);
    }
    virtual ~PktQueueServerMediaSubsession() {
        free(fFmtpLine);
    }

    virtual FramedSource* createNewStreamSource(unsigned /*clientSessionId*/,
                                                unsigned& estBitrate) {
        estBitrate = 4000;
        LOG("RTSP client connected, creating H264FramedSource from pkt_queue");
        FramedSource* src = H264FramedSource::createNew(envir(), fPktQueue);
        /* SPS+PPS are already in-band (prepended before first IDR frame).
         * Do NOT call setVPSandSPSandPPS — it would strip them and re-inject,
         * which causes NAL ordering issues with SEI before IDR. */
        return H264VideoStreamDiscreteFramer::createNew(envir(), src);
    }

    virtual RTPSink* createNewRTPSink(Groupsock* rtpGroupsock,
                                      unsigned char rtpPayloadType,
                                      FramedSource* /*inputSource*/) {
        return H264VideoRTPSink::createNew(envir(), rtpGroupsock, rtpPayloadType);
    }

    /* Override: always provide sprop-parameter-sets in SDP, even
     * before the framer is created. This is critical for the very
     * first DESCRIBE — otherwise ffplay never receives SPS/PPS. */
    virtual char const* getAuxSDPLine(RTPSink* /*rtpSink*/,
                                      FramedSource* /*inputSource*/) {
        return fFmtpLine;
    }

private:
    TQueue*      fPktQueue;
    u_int8_t     fRtpPayloadType;
    u_int8_t*    fSPS;
    unsigned     fSPSSize;
    u_int8_t*    fPPS;
    unsigned     fPPSSize;
    char*        fFmtpLine;
};

static void* event_loop_thread(void* arg) {
    TaskScheduler* scheduler = (TaskScheduler*)arg;
    scheduler->doEventLoop(&g_srv_stop);
    return NULL;
}

int rtsp_server_start(void* pkt_queue, int port, const char* stream_name,
                      uint8_t* sps, int sps_len, uint8_t* pps, int pps_len,
                      volatile int* stop_flag)
{
    (void)stop_flag;
    g_srv_stop = 0;

    TaskScheduler* scheduler = BasicTaskScheduler::createNew();
    if (!scheduler) { LOG_ERR("BasicTaskScheduler::createNew failed"); return -1; }

    UsageEnvironment* env = BasicUsageEnvironment::createNew(*scheduler);
    if (!env) { LOG_ERR("createNew UsageEnvironment failed"); delete scheduler; return -1; }

    RTSPServer* rtspServer = RTSPServer::createNew(*env, port, NULL);
    if (!rtspServer) {
        LOG_ERR("RTSPServer::createNew failed (port %d in use?)", port);
        env->reclaim();
        delete scheduler;
        return -1;
    }

    ServerMediaSession* sms = ServerMediaSession::createNew(
        *env, stream_name, stream_name,
        "live555 H.264 RTSP stream from RK3568 MPP");
    sms->addSubsession(PktQueueServerMediaSubsession::createNew(*env,
        (TQueue*)pkt_queue, 96, sps, (unsigned)sps_len, pps, (unsigned)pps_len));

    rtspServer->addServerMediaSession(sms);

    char* url = rtspServer->rtspURL(sms);
    LOG("RTSP server started: %s", url);
    delete[] url;

    int ret = pthread_create(&g_event_thread, NULL, event_loop_thread, scheduler);
    if (ret != 0) {
        LOG_ERR("pthread_create event loop failed");
        Medium::close(rtspServer);
        env->reclaim();
        delete scheduler;
        return -1;
    }

    LOG("RTSP event loop thread running");
    return 0;
}

void rtsp_server_stop(void) {
    g_srv_stop = 1;
    pthread_join(g_event_thread, NULL);
    LOG("RTSP server stopped");
}
