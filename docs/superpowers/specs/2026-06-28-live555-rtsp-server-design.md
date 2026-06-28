# live555 内嵌 RTSP 服务器替换 FFmpeg 推流 — 设计文档

**日期**: 2026-06-28
**硬件**: RK3568 + IMX415
**状态**: 已确认

---

## 1. 目标

将当前基于 FFmpeg avformat push 的推流方案，替换为程序内嵌 live555 RTSP 服务器，使客户端（ffplay/VLC）可直接拉流，无需额外 RTSP 服务器进程（如 mediamtx），同时去掉板子上的 FFmpeg 依赖。

---

## 2. 架构对比

```
原来:  v4l2_mpp_rtsp ──avformat push──► mediamtx(独立进程) ◄──ffplay拉流── PC

改后:  v4l2_mpp_rtsp (内置 live555 RTSP) ◄──ffplay/VLC 直连── PC
```

### 2.1 线程模型变化

```
原来:  capture线程 → frame_queue → enc线程 → pkt_queue → push线程(avformat阻塞I/O)
改后:  capture线程 → frame_queue → enc线程 → pkt_queue → live555 event loop(单线程)
```

push 线程消失。live555 event loop 接管所有网络调度，`doGetNextFrame()` 从 pkt_queue pop 数据。

---

## 3. 数据流

```
capture thread → frame_queue → encode thread → pkt_queue
                                                    │
                              live555 event loop ← doGetNextFrame() pop
                                    │
                              H.264 NAL → RTP packetize → UDP/TCP → 客户端
```

---

## 4. 模块变更

| 文件 | 动作 | 说明 |
|------|------|------|
| `ff_pusher.h/c` | **删除** | 不再需要 |
| `rtsp_server.h/c` | **新增** | live555 RTSP 服务器初始化、生命周期管理 |
| `H264FramedSource.h/c` | **新增** | 继承 live555 FramedSource，doGetNextFrame 从 pkt_queue 取数据 |
| `main.c` | **修改** | 去掉 `ff_pusher.h` 和 push_thread，替换为 `rtsp_server_start()` |
| `CMakeLists.txt` | **修改** | link `liveMedia groupsock BasicUsageEnvironment UsageEnvironment` 替换 avformat/avcodec/avutil |

其余文件（`config.h`, `ts_queue.h/c`, `v4l2_capture.h/c`, `mpp_encoder.h/c`）**不变**。

---

## 5. H264FramedSource 设计

继承 live555 的 `FramedSource`，核心是重写 `doGetNextFrame()`：

```
live555 调度器调用:
  H264VideoStreamFramer::doGetNextFrame()     ← live555 自带, RTP 分片/分帧
    └→ H264FramedSource::doGetNextFrame()     ← 自定义, 实现如下:
         ├─ tq_pop(pkt_queue)                  ← 阻塞等待编码线程产出
         ├─ memcpy → fTo / fFrameSize          ← 拷入 live555 缓冲区
         ├─ mpp_packet_deinit(pkt)             ← 释放 MPP 资源
         └─ afterGetting(this)                 ← 通知 live555 数据就绪
```

- `H264VideoStreamFramer` 是 live555 自带组件，自动处理 Annex-B 字节流解析、NAL 分割、FU-A 分片（超 MTU 时）、SPS/PPS out-of-band 管理
- SPS+PPS 通过 `H264VideoStreamFramer::setSPSandPPS()` 在 RTSP DESCRIBE 阶段注入

---

## 6. rtsp_server 接口

```c
// 启动 RTSP 服务器
// pkt_queue: 共享的编码包队列（生产者=编码线程，消费者=live555）
// port:      监听端口（默认 554）
// stream_name: 流路径，如 "live" → rtsp://IP:554/live
// sps/pps/sps_len/pps_len: SPS和PPS的独立NAL单元数据
// 返回 0 成功，内部启动 live555 event loop 线程
int rtsp_server_start(TQueue *pkt_queue, int port, const char *stream_name,
                      uint8_t *sps, int sps_len, uint8_t *pps, int pps_len, int *stop_flag);

// 优雅停止（设置 stop_flag，live555 停止接受新连接，关闭现有会话）
void rtsp_server_stop();
```

---

## 7. main.c 变更要点

- `#include "ff_pusher.h"` → `#include "rtsp_server.h"`
- 去掉 FFPusher 初始化和 push_thread
- 新增：从 `mpp_encoder_get_header()` 的 Annex-B 数据中提取独立的 SPS NAL 和 PPS NAL
- 调用 `rtsp_server_start(&pkt_queue, 554, "live", sps, sps_len, pps, pps_len, &g_stop)`
- SIGINT 时设 g_stop → `rtsp_server_stop()` → flush queues → join 线程

---

## 8. CMakeLists.txt 变更

```cmake
# 去掉:
#   avformat avcodec avutil

# 改为:
target_link_libraries(v4l2_mpp_rtsp
    rockchip_mpp
    liveMedia groupsock BasicUsageEnvironment UsageEnvironment
    pthread
)
```

板子依赖：`liblivemedia-dev` 或交叉编译 live555。

---

## 9. 依赖总结

| 原来 | 改后 |
|------|------|
| MPP + FFmpeg + mediamtx(额外进程) | MPP + live555 |
| 客户端 ffplay → mediamtx | 客户端 ffplay/VLC → 板子IP:554 直连 |

---

## 10. 非目标

- 不支持多路并发客户端（live555 默认支持，但不是本阶段重点）
- 不实现 RTSP 鉴权
- 不实现音频
