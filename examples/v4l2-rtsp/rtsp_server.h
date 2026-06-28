#ifndef RTSP_SERVER_H
#define RTSP_SERVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int  rtsp_server_start(void* pkt_queue, int port, const char* stream_name,
                       uint8_t* sps, int sps_len, uint8_t* pps, int pps_len,
                       volatile int* stop_flag);

void rtsp_server_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* RTSP_SERVER_H */
