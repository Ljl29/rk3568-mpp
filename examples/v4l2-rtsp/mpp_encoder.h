#ifndef MPP_ENCODER_H
#define MPP_ENCODER_H

#include <stddef.h>
#include <stdint.h>

typedef struct MppEncoder MppEncoder;

MppEncoder* mpp_encoder_open(int width, int height, int fps, int bps,
                             const char *rc_mode, int gop);

int  mpp_encoder_get_header(MppEncoder *enc, uint8_t **data, size_t *len);

int  mpp_encoder_encode(MppEncoder *enc, void *frame, void **packet);

void mpp_encoder_close(MppEncoder *enc);

#endif /* MPP_ENCODER_H */
