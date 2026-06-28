#ifndef MPP_ENCODER_H
#define MPP_ENCODER_H

#include <stddef.h>
#include <stdint.h>

typedef struct MppEncoder MppEncoder;

MppEncoder* mpp_encoder_open(int width, int height, int fps, int bps,
                             const char *rc_mode, int gop);

int  mpp_encoder_get_header(MppEncoder *enc, uint8_t **data, size_t *len);

/* ASYNC encode: put frame, then get packet (blocking) */
int  mpp_encoder_put_frame(MppEncoder *enc, void *frame);

/* Returns a packet whose data lives in enc's internal pkt_buf.
 * Caller must copy data out before the next get_packet call. */
int  mpp_encoder_get_packet(MppEncoder *enc, void **packet);

void mpp_encoder_close(MppEncoder *enc);

#endif /* MPP_ENCODER_H */
