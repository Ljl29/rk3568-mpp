# live555 内嵌 RTSP 服务器 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Replace FFmpeg avformat push with embedded live555 RTSP server, so clients (ffplay/VLC) can pull directly from `rtsp://板子IP:554/live` with no external server process.

**Architecture:** live555 is C++ — new modules are `.cpp` with `extern "C"` wrappers for C linkage from `main.c`. `H264FramedSource` (FramedSource subclass) uses non-blocking queue pop + delayed retry to avoid blocking the live555 event loop. `rtsp_server` wraps live555 boilerplate (RTSPServer, ServerMediaSession, event loop thread, shutdown).

**Tech Stack:** C (unchanged modules) + C++11 (new modules), librk_mpp, live555 (libliveMedia, libgroupsock, libBasicUsageEnvironment, libUsageEnvironment), libpthread

---

## File Map

| File | Action | Responsibility |
|------|--------|----------------|
| `ff_pusher.h` | Delete | — |
| `ff_pusher.c` | Delete | — |
| `H264FramedSource.h` | New | `extern "C"` interface for creating/destroying the source; C++ class declaration |
| `H264FramedSource.cpp` | New | FramedSource subclass: non-blocking pop from TQueue, delayed retry on empty |
| `rtsp_server.h` | New | `extern "C"` API: `rtsp_server_start()` / `rtsp_server_stop()` |
| `rtsp_server.cpp` | New | live555 RTSPServer + ServerMediaSession setup, event loop thread, shutdown |
| `main.c` | Modify | Remove ff_pusher/push_thread, add Annex-B SPS/PPS parsing, call `rtsp_server_start()` |
| `CMakeLists.txt` | Modify | Enable CXX, replace avformat/avcodec/avutil with live555 libs, add .cpp sources |

---

### Task 1: Delete ff_pusher, update CMakeLists.txt

**Files:**
- Delete: `examples/v4l2-rtsp/ff_pusher.h`, `examples/v4l2-rtsp/ff_pusher.c`
- Modify: `examples/v4l2-rtsp/CMakeLists.txt`

- [ ] **Step 1: Delete ff_pusher files**

```bash
rm examples/v4l2-rtsp/ff_pusher.h examples/v4l2-rtsp/ff_pusher.c
```

- [ ] **Step 2: Update CMakeLists.txt**

Replace the entire content of `CMakeLists.txt` with:

```cmake
cmake_minimum_required(VERSION 2.8)
project(v4l2_mpp_rtsp C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -O2 -g -std=gnu99")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2 -g")

# MPP public headers
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../../inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../osal/inc
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# live555 headers (adjust if installed elsewhere)
# Standard location: /usr/include/liveMedia, /usr/include/groupsock, etc.
# On target board with liblivemedia-dev, these are found automatically.

add_executable(v4l2_mpp_rtsp
    main.c
    v4l2_capture.c
    mpp_encoder.c
    ts_queue.c
    H264FramedSource.cpp
    rtsp_server.cpp
)

target_link_libraries(v4l2_mpp_rtsp
    rockchip_mpp
    liveMedia
    groupsock
    BasicUsageEnvironment
    UsageEnvironment
    pthread
)

install(TARGETS v4l2_mpp_rtsp RUNTIME DESTINATION bin)
```

- [ ] **Step 3: Verify CMake config**

```bash
cd examples/v4l2-rtsp/build && rm -rf * && cmake .. -DCMAKE_TOOLCHAIN_FILE=../../build/linux/aarch64/arm.linux.cross.cmake 2>&1
# Expected: Configuring done (will warn about missing .cpp files — expected)
```

---

### Task 2: H264FramedSource — live555 source from pkt_queue

**Files:**
- Create: `examples/v4l2-rtsp/H264FramedSource.h`
- Create: `examples/v4l2-rtsp/H264FramedSource.cpp`

- [ ] **Step 1: Write H264FramedSource.h**

```cpp
#ifndef H264_FRAMED_SOURCE_H
#define H264_FRAMED_SOURCE_H

#include <FramedSource.hh>

// Forward-declare C types (TQueue, MppPacket from ts_queue.h / mpp_packet.h)
struct TQueue;

// --- C++ class ---
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
    unsigned fPollIntervalUs;   // retry interval when queue is empty
};

// --- extern "C" wrapper for main.c ---
#ifdef __cplusplus
extern "C" {
#endif

// Create a H264FramedSource. Returns pointer castable to FramedSource*.
// The returned pointer is actually a H264FramedSource*.
void* h264_framed_source_new(void* env, void* pkt_queue);

#ifdef __cplusplus
}
#endif

#endif /* H264_FRAMED_SOURCE_H */
```

- [ ] **Step 2: Write H264FramedSource.cpp**

```cpp
#include "H264FramedSource.h"

// C headers (our existing modules)
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
    , fPollIntervalUs(10000)  // 10ms retry on empty queue
{
}

H264FramedSource::~H264FramedSource() {
}

void H264FramedSource::doGetNextFrame() {
    // Non-blocking pop from queue (timeout 0 = immediate return)
    void* raw_pkt = NULL;
    int ret = tq_pop(fPktQueue, &raw_pkt, 0);

    if (ret == 0 && raw_pkt != NULL) {
        MppPacket pkt = (MppPacket)raw_pkt;

        uint8_t* data = (uint8_t*)mpp_packet_get_pos(pkt);
        size_t   len  = mpp_packet_get_length(pkt);

        if (data && len > 0) {
            // Copy into live555's fTo buffer
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
        // Empty packet — fall through to retry
    }

    // Queue empty or flushed — schedule retry
    if (!fPktQueue->flushed) {
        nextTask() = envir().taskScheduler().scheduleDelayedTask(
            fPollIntervalUs, (TaskFunc*)deliverFrame0, this);
    } else {
        // Flushed — signal end of stream
        handleClosure();
    }
}

void H264FramedSource::doStopGettingFrames() {
    // live555 will delete us via Medium::close() — no explicit cleanup needed
    FramedSource::doStopGettingFrames();
}

void H264FramedSource::deliverFrame0(void* clientData) {
    ((H264FramedSource*)clientData)->deliverFrame();
}

void H264FramedSource::deliverFrame() {
    // Called from scheduled task — retry doGetNextFrame
    doGetNextFrame();
}

// --- extern "C" wrappers ---
extern "C" {

void* h264_framed_source_new(void* env, void* pkt_queue) {
    return H264FramedSource::createNew(
        *(UsageEnvironment*)env, (TQueue*)pkt_queue);
}

} // extern "C"
```

---

### Task 3: rtsp_server — live555 setup + event loop

**Files:**
- Create: `examples/v4l2-rtsp/rtsp_server.h`
- Create: `examples/v4l2-rtsp/rtsp_server.cpp`

- [ ] **Step 1: Write rtsp_server.h**

```cpp
#ifndef RTSP_SERVER_H
#define RTSP_SERVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Start embedded RTSP server.
 *
 * pkt_queue:   thread-safe queue fed by encoder thread
 * port:        RTSP listen port (typically 554)
 * stream_name: URL path, e.g. "live" → rtsp://<ip>:554/live
 * sps:         H.264 SPS NAL unit (without start code), owned by caller
 * sps_len:     length of SPS in bytes
 * pps:         H.264 PPS NAL unit (without start code), owned by caller
 * pps_len:     length of PPS in bytes
 * stop_flag:   pointer to global stop flag (volatile int set by SIGINT)
 *
 * Starts a live555 event loop thread internally.
 * Returns 0 on success, -1 on error.
 */
int  rtsp_server_start(void* pkt_queue, int port, const char* stream_name,
                       uint8_t* sps, int sps_len, uint8_t* pps, int pps_len,
                       volatile int* stop_flag);

/*
 * Gracefully shut down RTSP server.
 * Sets internal stop flag, waits for event loop thread to exit.
 * Safe to call if server was never started.
 */
void rtsp_server_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* RTSP_SERVER_H */
```

- [ ] **Step 2: Write rtsp_server.cpp**

```cpp
#include "rtsp_server.h"
#include "H264FramedSource.h"

// live555 headers
#include <liveMedia.hh>
#include <BasicUsageEnvironment.hh>
#include <GroupsockHelper.hh>

// C headers
extern "C" {
#include "ts_queue.h"
#include "config.h"
}

#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// --- Static globals for the event loop thread ---
static volatile int*  g_app_stop   = NULL;
static volatile int   g_srv_stop   = 0;
static char           g_watch_var  = 0;
static pthread_t      g_event_thread;

static void* event_loop_thread(void* arg) {
    TaskScheduler* scheduler = (TaskScheduler*)arg;

    // Run event loop until g_srv_stop is set
    while (!g_srv_stop) {
        scheduler->singleStep(100000);  // 100ms timeout
    }

    return NULL;
}

int rtsp_server_start(void* pkt_queue, int port, const char* stream_name,
                      uint8_t* sps, int sps_len, uint8_t* pps, int pps_len,
                      volatile int* stop_flag)
{
    g_app_stop = stop_flag;
    g_srv_stop = 0;

    // --- 1. Create scheduler and usage environment ---
    TaskScheduler* scheduler = BasicTaskScheduler::createNew();
    if (!scheduler) { LOG_ERR("BasicTaskScheduler::createNew failed"); return -1; }

    UsageEnvironment* env = BasicUsageEnvironment::createNew(*scheduler);
    if (!env) { LOG_ERR("createNew UsageEnvironment failed"); delete scheduler; return -1; }

    // --- 2. Create RTSPServer ---
    RTSPServer* rtspServer = RTSPServer::createNew(*env, port, NULL);
    if (!rtspServer) {
        LOG_ERR("RTSPServer::createNew failed (port %d in use?)", port);
        env->reclaim();
        delete scheduler;
        return -1;
    }

    // --- 3. Build the stream ---
    // H264FramedSource (our custom source from pkt_queue)
    FramedSource* videoSource = (FramedSource*)h264_framed_source_new(env, pkt_queue);
    if (!videoSource) {
        LOG_ERR("h264_framed_source_new failed");
        Medium::close(rtspServer);
        env->reclaim();
        delete scheduler;
        return -1;
    }

    // H264VideoStreamDiscreteFramer: splits Annex-B into NAL units for RTP packetization
    H264VideoStreamDiscreteFramer* framer =
        H264VideoStreamDiscreteFramer::createNew(*env, videoSource);

    // Set SPS/PPS for out-of-band signaling (RTSP DESCRIBE → sprop-parameter-sets)
    // live555 expects raw NAL data (no start code prefix)
    framer->setSPSandPPS(*env, sps, (unsigned)sps_len, pps, (unsigned)pps_len);

    // RTPSink: H.264 RTP packetizer
    RTPSink* videoSink = H264VideoRTPSink::createNew(*env, NULL, 96);
    videoSink->setStreamSocketSize(512 * 1024);   // 512KB socket buffer

    // --- 4. Create ServerMediaSession and add subsession ---
    ServerMediaSession* sms = ServerMediaSession::createNew(
        *env, stream_name, stream_name,
        "live555 H.264 RTSP stream from RK3568 MPP");
    sms->addSubsession(PassiveServerMediaSubsession::createNew(*videoSink, NULL));

    rtspServer->addServerMediaSession(sms);

    char* url = rtspServer->rtspURL(sms);
    LOG("RTSP server started: %s", url);
    delete[] url;

    // --- 5. Launch event loop in a dedicated thread ---
    int ret = pthread_create(&g_event_thread, NULL, event_loop_thread, scheduler);
    if (ret != 0) {
        LOG_ERR("pthread_create event loop failed");
        Medium::close(rtspServer);
        env->reclaim();
        delete scheduler;
        return -1;
    }

    // --- 6. Monitor stop_flag in main's sleep loop ---
    // The application main loop checks *stop_flag every 5s.
    // When it becomes 1, main calls rtsp_server_stop().
    // We also check stop_flag here via a scheduled task
    // (handled externally by main loop — see Task 4).

    LOG("RTSP event loop thread running");
    return 0;
}

void rtsp_server_stop(void) {
    g_srv_stop = 1;
    pthread_join(g_event_thread, NULL);
    LOG("RTSP server stopped");
}
```

---

### Task 4: main.c — swap ff_pusher for rtsp_server

**Files:**
- Modify: `examples/v4l2-rtsp/main.c`

- [ ] **Step 1: Read the current main.c**

Read `examples/v4l2-rtsp/main.c` to understand the full context.

- [ ] **Step 2: Replace includes**

Replace this line:
```c
#include "ff_pusher.h"
```
with:
```c
#include "rtsp_server.h"
```

- [ ] **Step 3: Add Annex-B SPS/PPS extraction helper**

Insert after the `print_stats()` function (before `usage()`):

```c
/*
 * Parse Annex-B byte stream to extract individual SPS and PPS NAL units.
 * Input:  Annex-B data from mpp_encoder_get_header() (SPS+IDR+PPS concatenated)
 * Output: sps/pps pointers point into the input buffer (no copy).
 *         sps_len/pps_len set to the NAL unit sizes (without start code).
 * Returns 0 on success, -1 if SPS or PPS not found.
 */
static int extract_sps_pps(uint8_t *annexb_data, size_t annexb_len,
                           uint8_t **sps, int *sps_len,
                           uint8_t **pps, int *pps_len) {
    uint8_t *p = annexb_data;
    size_t remaining = annexb_len;
    *sps = *pps = NULL;
    *sps_len = *pps_len = 0;

    while (remaining > 3) {
        /* Find start code: 0x00 0x00 0x00 0x01 or 0x00 0x00 0x01 */
        int sc_len = 0;
        if (p[0] == 0x00 && p[1] == 0x00 && p[2] == 0x00 && p[3] == 0x01)
            sc_len = 4;
        else if (p[0] == 0x00 && p[1] == 0x00 && p[2] == 0x01)
            sc_len = 3;
        else {
            p++; remaining--; continue;
        }
        p += sc_len; remaining -= (size_t)sc_len;

        /* Find next start code or end to determine NAL length */
        uint8_t *next = p;
        size_t nal_len = 0;
        while (nal_len + 2 < remaining) {
            if (next[0] == 0x00 && next[1] == 0x00 &&
                (next[2] == 0x01 || (nal_len + 3 < remaining && next[2] == 0x00 && next[3] == 0x01)))
                break;
            next++; nal_len++;
        }
        if (nal_len == 0) { nal_len = remaining; }

        /* Identify NAL type (lower 5 bits of first byte) */
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
```

- [ ] **Step 4: Replace FFPusher init and push_thread launch in main()**

Find the block (approximately lines 288–301 in current main.c):

```c
    uint8_t *hdr_data = NULL;
    size_t   hdr_len  = 0;
    mpp_encoder_get_header(enc, &hdr_data, &hdr_len);
    if (!hdr_data || hdr_len == 0) {
        LOG_ERR("Failed to get SPS/PPS header");
        mpp_encoder_close(enc); v4l2_capture_close(cam); return EXIT_FAILURE;
    }

    FFPusher *pusher = ff_pusher_open(cfg.url, cfg.width, cfg.height, cfg.fps,
                                      hdr_data, hdr_len);
    if (!pusher) {
        LOG_ERR("Failed to open push connection to %s", cfg.url);
        mpp_encoder_close(enc); v4l2_capture_close(cam); return EXIT_FAILURE;
    }
```

Replace with:

```c
    /* Get SPS+PPS header and extract individual NAL units for live555 */
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
```

- [ ] **Step 5: Replace thread launch block**

Find the section that launches `pthread_create` for `t_push` (and `t_enc`, `t_cap`). Replace with:

```c
    /* Launch capture and encode threads (push thread is replaced by live555) */
    pthread_t t_cap, t_enc;
    pthread_create(&t_cap,  NULL, capture_thread, &tc);
    pthread_create(&t_enc,  NULL, encode_thread,  &tc);

    /* Start RTSP server (launches its own event loop thread) */
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
```

- [ ] **Step 6: Replace shutdown sequence**

Find the shutdown block (flush → join → CLEANUP section). Replace:

```c
    LOG("Shutting down...");
    rtsp_server_stop();
    tq_flush(&frame_queue);
    tq_flush(&pkt_queue);

    /* Join threads (only capture and encode — live555 stopped above) */
    pthread_join(t_cap,  NULL);
    pthread_join(t_enc,  NULL);

    print_stats(&tc);
    LOG("Pipeline stopped.");

CLEANUP:
    tq_destroy(&frame_queue);
    tq_destroy(&pkt_queue);
    tq_destroy(&done_queue);
    mpp_encoder_close(enc);
    v4l2_capture_close(cam);
    return 0;
```

- [ ] **Step 7: Remove push-specific fields from ThreadCtx and print_stats**

In `ThreadCtx` struct, remove these 4 lines:
```c
    FFPusher     *pusher;
    ...
    int64_t        push_frames;
    int64_t        push_errors;
    int64_t        push_bytes;
```

Replace `print_stats()` with:
```c
static void print_stats(ThreadCtx *tc) {
    LOG("Stats: capture=%lld(+%lld drops) encode=%lld(+%lld errs) | fq=%d pq=%d",
        (long long)tc->capture_frames, (long long)tc->capture_drops,
        (long long)tc->encode_frames, (long long)tc->encode_errors,
        tq_count(tc->frame_queue), tq_count(tc->pkt_queue));
}
```

- [ ] **Step 8: Remove --url from CLI**

In `usage()`: remove line `"  --url        URL    RTSP/RTMP push URL (REQUIRED)\n"`.

In `long_opts[]`: remove `{"url",       required_argument, 0, 'u'},`.

In `parse_args()`: remove `int url_set = 0;` declaration, remove `add_set = 1;` from case 'u', remove case 'u' entirely, remove the `if (!url_set) { LOG_ERR("--url is required"); ... return -1; }` block at end.

In `config.h` AppConfig: remove `char url[256];` field.
In `config.h` config_set_defaults(): remove any reference to `c->url`.

---

### Task 5: config.h — remove unused url field

**Files:**
- Modify: `examples/v4l2-rtsp/config.h`

- [ ] **Step 1: Remove url from AppConfig**

In `config.h`, remove line:
```c
    char    url[256];          /* rtsp://... or rtmp://... */
```

And in `config_set_defaults()`, remove the final `--url` requirement comment (url is no longer a CLI arg).

---

### Task 6: Build verification

- [ ] **Step 1: List all source files**

```bash
ls -la examples/v4l2-rtsp/
# Expected: CMakeLists.txt  config.h  ts_queue.h/c  v4l2_capture.h/c
#           mpp_encoder.h/c  H264FramedSource.h/cpp  rtsp_server.h/cpp  main.c
```

- [ ] **Step 2: CMake configure**

```bash
cd examples/v4l2-rtsp/build && rm -rf * && cmake .. -DCMAKE_TOOLCHAIN_FILE=../../build/linux/aarch64/arm.linux.cross.cmake 2>&1
# Expected: Configuring done. Generate done.
```

- [ ] **Step 3: Build**

```bash
make -j$(nproc)
# Expected: all objects built, v4l2_mpp_rtsp linked
```

- [ ] **Step 4: Run and test**

```bash
# On the RK3568 board:
./v4l2_mpp_rtsp --device /dev/video0
# Expected: "RTSP server started: rtsp://<ip>:554/live"

# On a PC (same network):
ffplay rtsp://<board-ip>:554/live
# Expected: video playback
```
