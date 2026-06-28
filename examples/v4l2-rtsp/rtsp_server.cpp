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

    FramedSource* videoSource = (FramedSource*)h264_framed_source_new(env, pkt_queue);
    if (!videoSource) {
        LOG_ERR("h264_framed_source_new failed");
        Medium::close(rtspServer);
        env->reclaim();
        delete scheduler;
        return -1;
    }

    H264VideoStreamDiscreteFramer* framer =
        H264VideoStreamDiscreteFramer::createNew(*env, videoSource);
    framer->setVPSandSPSandPPS(NULL, 0, sps, (unsigned)sps_len, pps, (unsigned)pps_len);

    RTPSink* videoSink = H264VideoRTPSink::createNew(*env, NULL, 96);

    ServerMediaSession* sms = ServerMediaSession::createNew(
        *env, stream_name, stream_name,
        "live555 H.264 RTSP stream from RK3568 MPP");
    sms->addSubsession(PassiveServerMediaSubsession::createNew(*videoSink, NULL));

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
