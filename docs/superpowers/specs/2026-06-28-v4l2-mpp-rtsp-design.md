# V4L2 采集 → MPP H.264 编码 → RTSP/RTMP 推流 — 设计文档

**日期**: 2026-06-28
**硬件**: RK3568 + IMX415 MIPI CSI
**状态**: 已确认

---

## 1. 目标与范围

### 1.1 项目目标

在 RK3568 平台开发一个可稳定运行的 C 程序，打通 **V4L2 采集 → MPP H.264 硬编码 → FFmpeg RTSP/RTMP 推流** 全链路。同时深入理解 MPP 编解码框架的内部 pipeline（MPI → codec parser → HAL → 硬件寄存器）。

### 1.2 关键参数

| 参数 | 值 |
|------|---|
| 采集源 | IMX415 MIPI CSI → `/dev/videoX`, NV12 (YUV420SP) |
| 分辨率 | 1920×1080 |
| 帧率 | 30 fps |
| 编码格式 | H.264 (MPP_VIDEO_CodingAVC) |
| 编码配置 | High Profile, Level 4.0, CABAC, 4Mbps CBR, GOP=60 |
| 推流协议 | RTSP + RTMP 双协议 |
| 集成方式 | FFmpeg libavformat 库内嵌 |

### 1.3 非目标 (不在本阶段范围)

- H.265/HEVC 编码支持（后续迭代）
- 多通道编码
- 音频采集与编码
- ONVIF 协议支持
- WebRTC 推流

---

## 2. 项目结构

```
mpp-release/
└── examples/
    └── v4l2-rtsp/
        ├── CMakeLists.txt          # 独立 CMake 构建配置
        ├── main.c                  # 入口 + 命令行解析 + 信号 + 统计
        ├── config.h                # 公共配置宏、共享数据结构
        ├── v4l2_capture.h          # V4L2 采集模块接口
        ├── v4l2_capture.c          # V4L2 采集模块实现
        ├── mpp_encoder.h           # MPP H.264 编码模块接口
        ├── mpp_encoder.c           # MPP H.264 编码模块实现
        ├── ff_pusher.h             # FFmpeg 推流模块接口
        ├── ff_pusher.c             # FFmpeg RTSP/RTMP 推流模块实现
        ├── ts_queue.h              # 线程安全 bounded queue 接口
        └── ts_queue.c              # 线程安全 bounded queue 实现
```

共 10 个文件。每个模块接口清晰、边界明确，可独立理解和测试。

---

## 3. 整体架构

### 3.1 数据流

```
IMX415 sensor → MIPI CSI → ISP(硬件) → /dev/videoX (NV12)
    │
    ▼ (V4L2 mmap / VIDIOC_DQBUF → MppBuffer wrapping)
[MppFrame + MppBuffer] → FrameQueue → [MPP encode_put_frame]
    │
    ▼ (MPP 内部: codec parser → HAL register gen → VEPU 硬件)
[MppPacket (H.264 Annex-B NAL units)] → PktQueue → [FFmpeg avformat muxer]
    │
    ▼ (av_interleaved_write_frame)
[RTSP (UDP/TCP) / RTMP (TCP)] → 网络
```

### 3.2 三线程架构

```
Thread 1: V4L2 Capture        Thread 2: MPP Encode         Thread 3: FFmpeg Push
┌──────────────────┐         ┌──────────────────┐         ┌──────────────────┐
│ VIDIOC_DQBUF     │         │ fq.pop(frame)    │         │ pq.pop(pkt)      │
│ (阻塞等帧)        │         │                  │         │                  │
│       │          │         │ encode_put_frame │         │ av_interleaved   │
│       ▼          │         │       │          │         │   _write_frame() │
│ 提取 CamFrame    │         │ poll(OUTPUT)     │         │       │          │
│       │          │         │ dequeue(OUTPUT)  │         │ 网络发送(TCP/UDP)│
│       ▼          │         │ encode_get_packet│         │       │          │
│ fq.push(frame) ──┼──►     │ pq.push(pkt) ────┼──►     │ 统计(帧率/码率)   │
│       │          │         │       │          │         │       │          │
│ 检查 stop_flag   │         │ 检查 stop_flag   │         │ 检查 stop_flag   │
│ 循环             │         │ 循环              │         │ 循环             │
└──────────────────┘         └──────────────────┘         └──────────────────┘
     frame_queue                  pkt_queue
  (bounded, cap=5)            (bounded, cap=12)
```

### 3.3 模块清单

| 模块 | 文件 | 职责 | 外部依赖 |
|------|------|------|---------|
| V4L2 采集 | `v4l2_capture.c/h` | 打开/配置 V4L2 设备、mmap 映射、DQ/BUF、帧率控制 | libc, `<linux/videodev2.h>` |
| MPP 编码 | `mpp_encoder.c/h` | 创建编码器、配置 H.264、输入 Frame 输出 Packet | librockchip_mpp |
| FFmpeg 推流 | `ff_pusher.c/h` | avformat muxing、RTSP/RTMP URL 识别与网络输出 | libavformat, libavcodec, libavutil |
| 线程安全队列 | `ts_queue.c/h` | 固定大小 bounded queue、blocking push/pop、超时、flush | libpthread |
| 主程序 | `main.c` | 参数解析、模块初始化、线程启动/停止、SIG 处理、统计日志 | 以上全部 |
| 配置 | `config.h` | 公共宏定义、全局配置结构体、默认值 | 无 |

---

## 4. MPP 编码器内部架构

### 4.1 四层架构

MPP 编码器内部按职责分为四层，对应调用流程如下：

```
应用程序
   │
   ├─ mpp_create + mpp_init(ctx, ENC, AVC)    → 创建编码器上下文
   │
   ├─ mpp_enc_cfg_init                       → 初始化配置结构
   ├─ prep: 配置 width/height/stride/format   → 预处理配置层
   ├─ rc:   配置 CBR/VBR/FIXQP, bps, fps      → 码率控制层
   ├─ codec:配置 H.264 profile/level/cabac    → 编码器语法层
   ├─ mpi->control(SET_CFG)                  → 下发全部配置
   │
   ├─ mpi->control(GET_HDR_SYNC)             → 获取 SPS+PPS+IDR header
   │     ↓ 保存为 FFmpeg AVCodecParameters.extradata
   │
   └─ 编码循环:
       encode_put_frame(frame)
         → codec parser (h264e_sps/pps/slice/dpb/sei)
           → HAL register gen (hal_h264e_api_v2.c)
             → VEPU 硬件编码 + IRQ 回调
       encode_get_packet(packet) → H.264 Annex-B 码流
```

### 4.2 H.264 编码器语法元素映射

| 源文件 (mpp/codec/enc/h264/) | 负责的 H.264 语法 | 关键知识点 |
|------|------|------|
| `h264e_sps.c` | SPS (Sequence Parameter Set) | profile_idc, level_idc, pic_width/height_in_mbs, frame_cropping, log2_max_frame_num, pic_order_cnt_type, num_ref_frames |
| `h264e_pps.c` | PPS (Picture Parameter Set) | pic_init_qp_minus26, entropy_coding_mode_flag (CAVLC/CABAC), deblocking_filter_control |
| `h264e_slice.c` | Slice Header | slice_type (I/P), frame_num, pic_order_cnt, slice_qp_delta, ref_pic_list_modification |
| `h264e_dpb.c` | DPB (Decoded Picture Buffer) | 参考帧管理、long-term/short-term ref、MMCO commands |
| `h264e_sei.c` | SEI (Supplemental Enhancement Info) | buffering_period, pic_timing, user_data |

### 4.3 编码器配置 (1080p30 H.264 High Profile)

```c
// prep
mpp_enc_cfg_set_s32(cfg, "prep:width",       1920);
mpp_enc_cfg_set_s32(cfg, "prep:height",      1080);
mpp_enc_cfg_set_s32(cfg, "prep:hor_stride",  1920);
mpp_enc_cfg_set_s32(cfg, "prep:ver_stride",  1080);
mpp_enc_cfg_set_s32(cfg, "prep:format",      MPP_FMT_YUV420SP);

// rc
mpp_enc_cfg_set_s32(cfg, "rc:mode",          MPP_ENC_RC_MODE_CBR);
mpp_enc_cfg_set_s32(cfg, "rc:bps_target",    4000000);
mpp_enc_cfg_set_s32(cfg, "rc:bps_max",       4500000);
mpp_enc_cfg_set_s32(cfg, "rc:bps_min",       3500000);
mpp_enc_cfg_set_s32(cfg, "rc:fps_in_num",    30);
mpp_enc_cfg_set_s32(cfg, "rc:fps_in_denorm", 1);
mpp_enc_cfg_set_s32(cfg, "rc:fps_out_num",   30);
mpp_enc_cfg_set_s32(cfg, "rc:fps_out_denorm",1);
mpp_enc_cfg_set_s32(cfg, "rc:gop",           60);

// codec
mpp_enc_cfg_set_s32(cfg, "codec:type",       MPP_VIDEO_CodingAVC);
mpp_enc_cfg_set_s32(cfg, "h264:profile",     100);
mpp_enc_cfg_set_s32(cfg, "h264:level",       40);
mpp_enc_cfg_set_s32(cfg, "h264:cabac_en",    1);
mpp_enc_cfg_set_s32(cfg, "h264:cabac_idc",   0);
mpp_enc_cfg_set_s32(cfg, "h264:trans8x8",    1);

// header mode: 每个 IDR 帧前附加 SPS/PPS (RTSP 客户端随时加入所需)
mpi->control(ctx, MPP_ENC_SET_HEADER_MODE, &(RK_U32){MPP_ENC_HEADER_MODE_EACH_IDR});
```

---

## 5. 多线程队列模型

### 5.1 Bounded Queue 接口

```c
typedef struct {
    void          **bufs;        // 环形缓冲区
    int             capacity;    // 容量
    int             head;        // 写位置
    int             tail;        // 读位置
    int             count;       // 当前元素数
    pthread_mutex_t mutex;
    pthread_cond_t  not_empty;
    pthread_cond_t  not_full;
    int             flushed;     // flush 标记，用于优雅退出
} TQueue;

int  tq_init(TQueue *q, int capacity);
int  tq_push(TQueue *q, void *item, int timeout_ms);   // 阻塞写入 (超时返回)
int  tq_pop(TQueue *q, void **item, int timeout_ms);   // 阻塞读出 (超时返回)
void tq_flush(TQueue *q);                              // 唤醒所有等待
void tq_destroy(TQueue *q);                            // 释放资源
```

### 5.2 背压与丢帧策略

- **正常状态**: frame_queue 3/5、pkt_queue 2/12，各环节匹配，无丢帧
- **推流慢 (网络波动)**: pkt_queue 堆满 → 编码线程 push 超时 100ms → **丢弃当前 pkt**（记录丢帧计数，优先保障低延迟）
- **编码慢 (极少发生)**: frame_queue 堆满 → 采集线程 push 超时 100ms → **丢弃最旧帧**（避免积累旧帧导致延迟增大）
- **命令行控制**: `--drop 0` 切换为阻塞模式（不丢帧但延迟可能增大），`--drop 1` 为丢帧模式（默认）

### 5.3 延迟分析

| 环节 | 典型延迟 | 说明 |
|------|---------|------|
| V4L2 采集 | 1–2 ms | mmap 零拷贝 + DMA, 延迟在传感器曝光 |
| MPP 编码 | 15–25 ms | RK3568 VEPU 硬件编码 1080p30 |
| FFmpeg muxing | <1 ms | 纯软件，仅打包 NAL 单元 |
| 网络发送 | 5–30 ms | 取决于网络状况和 RTP/RTMP 缓冲 |
| **端到端预估** | **~40–60 ms** | 主要瓶颈在硬件编码单元 |

---

## 6. FFmpeg 推流模块

### 6.1 初始化流程

```
avformat_network_init()
avformat_alloc_output_context2(&ctx, NULL, NULL, url)
  → URL "rtsp://..." 自动匹配 rtsp muxer
  → URL "rtmp://..." 自动匹配 flv muxer

avformat_new_stream(ctx, NULL)
  → st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO
  → st->codecpar->codec_id   = AV_CODEC_ID_H264
  → st->time_base            = {1, 90000}  (90kHz, H.264 标准)

extradata = 从 MPP GET_HDR_SYNC 获取的 SPS+PPS (Annex-B 格式)
  → st->codecpar->extradata      = extradata
  → st->codecpar->extradata_size = extradata_len

avformat_write_header(ctx, NULL)  → SDP 协商 / RTMP handshake
```

### 6.2 推流循环

```c
while (!stop) {
    pq.pop(&pkt, 100);              // 从 PktQueue 取 MppPacket，100ms 超时
    if (flushed) break;

    av_init_packet(&avpkt);
    avpkt.data         = mpp_packet_get_data(pkt);
    avpkt.size         = mpp_packet_get_length(pkt);
    avpkt.stream_index = video_st->index;

    // 时间戳: 基于帧序号计算 (90kHz 时钟)
    int64_t pts_90k = frame_count * 90000 / 30;
    avpkt.pts = pts_90k;
    avpkt.dts = pts_90k;

    av_interleaved_write_frame(ctx, &avpkt);  // 实际网络发送
    av_packet_unref(&avpkt);

    mpp_packet_deinit(&pkt);
    frame_count++;
}
```

### 6.3 模块接口

```c
typedef struct FFPusher FFPusher;

// 打开推流连接
// url: rtsp://... 或 rtmp://...
// extradata/extradata_len: H.264 SPS+PPS (Annex-B 格式)
// 返回 NULL 表示失败
FFPusher* ff_pusher_open(const char *url, int width, int height, int fps,
                         uint8_t *extradata, size_t extradata_len);

// 写入一帧编码数据 (阻塞直到网络发送完成)
// 返回 0 成功，非 0 失败
int ff_pusher_write(FFPusher *p, MppPacket packet);

// 关闭推流，发送 trailer
int ff_pusher_close(FFPusher *p);
```

---

## 7. V4L2 采集模块

### 7.1 设计要点

- 基于 MPP 已有 `utils/camera_source.c` 的实现逻辑，封装为独立模块
- 使用 V4L2 mmap 零拷贝方式，将内核缓冲区映射到用户空间
- IMX415 通过 MIPI CSI + ISP 后输出 NV12 格式
- 采集到的帧包装为 `MppBuffer`，通过 FrameQueue 传递给编码线程

### 7.2 接口

```c
typedef struct V4L2Capture V4L2Capture;

// 打开 V4L2 设备并配置格式
// dev:    设备路径，如 "/dev/video0"
// width/height: 目标分辨率
// fps:    目标帧率 (用于 V4L2 内部定时)
// buf_cnt: 内核缓冲区数量 (推荐 4)
V4L2Capture* v4l2_capture_open(const char *dev, int width, int height,
                               int fps, int buf_cnt);

// 获取一帧 (阻塞直到有帧可用)
// 返回 0 成功，<0 失败。调用者获得 frame 和 buffer 的所有权
int v4l2_capture_get_frame(V4L2Capture *c, MppBuffer *buf, MppFrame *frame);

// 关闭设备并释放资源
int v4l2_capture_close(V4L2Capture *c);
```

---

## 8. MPP 编码模块

### 8.1 接口

```c
typedef struct MppEncoder MppEncoder;

// 创建并初始化 MPP 编码器
// width/height: 输入分辨率
// fps: 帧率
// bps: 目标码率 (bps)
// rc_mode: "cbr" / "vbr" / "fixqp"
// gop: GOP 长度
MppEncoder* mpp_encoder_open(int width, int height, int fps, int bps,
                              const char *rc_mode, int gop);

// 获取 SPS+PPS header data (Annex-B 格式)
// 调用者不需要释放 data 指向的内存 (由 encoder 内部管理)
int mpp_encoder_get_header(MppEncoder *enc, uint8_t **data, size_t *len);

// 编码一帧
// frame: 输入 (MppFrame, 需包含 MppBuffer)
// packet: 输出 (MppPacket, H.264 Annex-B 格式)
// 返回 0 成功
int mpp_encoder_encode(MppEncoder *enc, MppFrame frame, MppPacket *packet);

// 关闭编码器
int mpp_encoder_close(MppEncoder *enc);
```

---

## 9. 主程序

### 9.1 命令行接口

```
v4l2_mpp_rtsp [OPTIONS]

--device     /dev/videoX    V4L2 设备路径 (必需)
--width      1920           采集宽度 (默认 1920)
--height     1080           采集高度 (默认 1080)
--fmt        nv12           输入格式 (默认 nv12, MPP_FMT_YUV420SP)
--fps        30             目标帧率 (默认 30)
--bitrate    4000000        目标码率 bps (默认 4000000)
--rc-mode    cbr            码控模式: cbr/vbr/fixqp (默认 cbr)
--gop        60             GOP 长度 (默认 60)
--profile    high           H.264 profile: high/main/baseline (默认 high)
--url        rtsp://...     推流 URL，RTSP 或 RTMP (必需)
--queue-len  5              帧队列深度 (默认 5)
--drop       1              丢帧策略: 0=阻塞, 1=丢帧 (默认 1)
--verbose    0              详细日志: 0=关闭, 1=每帧统计 (默认 0)
--help                      打印帮助信息
```

### 9.2 主流程伪代码

```c
int main(int argc, char **argv) {
    // 1. 解析命令行参数 → AppConfig
    AppConfig cfg = parse_args(argc, argv);

    // 2. 初始化模块
    V4L2Capture *cam   = v4l2_capture_open(cfg.device, cfg.width, cfg.height, cfg.fps, 4);
    MppEncoder  *enc   = mpp_encoder_open(cfg.width, cfg.height, cfg.fps, cfg.bitrate, cfg.rc_mode, cfg.gop);
    uint8_t *hdr; size_t hdr_len;
    mpp_encoder_get_header(enc, &hdr, &hdr_len);
    FFPusher    *pusher = ff_pusher_open(cfg.url, cfg.width, cfg.height, cfg.fps, hdr, hdr_len);

    TQueue frame_queue, pkt_queue;
    tq_init(&frame_queue, cfg.queue_len);
    tq_init(&pkt_queue, 12);

    // 3. 注册 SIGINT/SIGTERM → 设置 g_stop = 1

    // 4. 启动线程
    pthread_t t_capture, t_encode, t_push;
    pthread_create(&t_capture, NULL, capture_thread, &args);
    pthread_create(&t_encode,  NULL, encode_thread,  &args);
    pthread_create(&t_push,    NULL, push_thread,    &args);

    // 5. 主线程: 定时打印统计 (帧率, 码率, 队列深度, 丢帧数)

    // 6. 等待信号 → g_stop = 1 → flush queue → join 线程

    // 7. 清理
    tq_destroy(&frame_queue); tq_destroy(&pkt_queue);
    ff_pusher_close(pusher);
    mpp_encoder_close(enc);
    v4l2_capture_close(cam);
    return 0;
}
```

---

## 10. 构建系统

### 10.1 CMakeLists.txt 核心内容

```cmake
cmake_minimum_required(VERSION 2.8)
project(v4l2_mpp_rtsp)
set(CMAKE_C_STANDARD 99)

# 交叉编译工具链由上层 -DCMAKE_TOOLCHAIN_FILE= 指定
# 此处引用项目当前的 arm.linux.cross.cmake

include_directories(
    ${CMAKE_SOURCE_DIR}/../../inc    # MPP 公共头文件: rk_mpi.h, mpp_frame.h 等
    ${CMAKE_SOURCE_DIR}              # 本项目头文件
)

add_executable(v4l2_mpp_rtsp
    main.c
    v4l2_capture.c
    mpp_encoder.c
    ff_pusher.c
    ts_queue.c
)

target_link_libraries(v4l2_mpp_rtsp
    rockchip_mpp        # 或 ${RK_MPP_LIB}
    avformat
    avcodec
    avutil
    pthread
)

install(TARGETS v4l2_mpp_rtsp RUNTIME DESTINATION bin)
```

### 10.2 编译命令

```bash
# 在开发板上
cd examples/v4l2-rtsp
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../../build/linux/aarch64/arm.linux.cross.cmake
make -j$(nproc)

# 运行
./v4l2_mpp_rtsp --device /dev/video0 --url rtsp://192.168.1.100:8554/live
```

---

## 11. 错误处理策略

| 场景 | 处理方式 |
|------|---------|
| V4L2 设备打开失败 | 打印 errno 含义，退出 EXIT_FAILURE |
| V4L2 采集超时 (3 秒无帧) | 线程内打印 warning，记录超时次数，>10 次则设置 g_stop |
| MPP 编码返回错误 | 编码线程打印 MPP 错误码，丢弃当前帧，继续处理下一帧 |
| FFmpeg 推流连接断开 | 推流线程设置 g_stop，通知其他线程退出 |
| SIGINT/SIGTERM | 设置 g_stop → flush 两个队列 → join 所有线程 → 依次 deinit |
| 内存分配失败 | 打印错误日志，当前模块 init 返回 NULL，主程序依次清理已 init 的资源 |

---

## 12. 性能与优化路径

### 12.1 初始版本目标

- 1080p30 CBR 4Mbps 稳定推流
- 端到端延迟 < 100ms
- 无内存泄漏 (Valgrind/memstat 验证)
- 24 小时持续运行不掉线

### 12.2 后续优化方向

| 优化项 | 手段 | 预期收益 |
|------|------|---------|
| 零拷贝增强 | V4L2 dma-buf fd 直传 MPP，跳过 CPU 拷贝 | 降低 2-5ms 延迟 |
| Dynamic GOP | 根据场景复杂度动态调整 GOP 长度 | 码率利用率提升 10-15% |
| Profile 切换 | 低延迟场景自动切换 Baseline Profile | 编码延迟降低 5-10ms |
| RTP 层优化 | 自定义 RTP packetization 替代 avformat (可选) | 延迟可进一步降低至 30-40ms |
| 硬件时间戳 | 利用 V4L2 timestamp 精确追踪 pipeline 延迟 | 精确定位瓶颈 |
| QP 调优 | 调整 qp_init/qp_max/qp_min 提升画质或降低码率 | 画质/码率平衡 |

---

## 13. 学习路径指引

本项目的代码结构按学习深度递进组织：

1. **ts_queue.c** — 最独立的基础设施，不依赖 MPP/FFmpeg，先理解线程同步模型
2. **v4l2_capture.c** — 标准 Linux V4L2 API，理解 mmap/DMA 零拷贝
3. **mpp_encoder.c** — MPP MPI 异步接口 (`encode_put_frame` + `encode_get_packet`)，理解 encoder cfg 四层配置流程
4. **ff_pusher.c** — FFmpeg libavformat 内部 muxing API，理解 extradata、time_base、muxer 选择
5. **main.c** — 将所有模块串联，理解多线程 pipeline 协调、统计、退避策略

深入 MPP 内部的路径:
6. **h264e_sps.c / h264e_pps.c / h264e_slice.c** — H.264 语法元素在 MPP 中的生成
7. **hal_h264e_api_v2.c** — HAL 层如何将 syntax 转译为 VEPU 硬件寄存器
8. **mpi_enc_test.c** — MPP 官方 demo，对比理解 async 和 sync 两种 API 使用方式
