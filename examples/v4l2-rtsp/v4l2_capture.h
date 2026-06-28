#ifndef V4L2_CAPTURE_H
#define V4L2_CAPTURE_H

#include <stddef.h>
#include <linux/videodev2.h>

typedef struct V4L2Capture V4L2Capture;

/*
 * Open and configure V4L2 device.
 *  dev     : device path, e.g. "/dev/video0"
 *  width   : desired width
 *  height  : desired height
 *  fps     : desired frame rate (used in V4L2_CID or set by driver)
 *  buf_cnt : number of kernel buffers to request (typically 4)
 *  Returns NULL on failure (prints error to stderr).
 */
V4L2Capture* v4l2_capture_open(const char *dev, int width, int height,
                               int fps, int buf_cnt);

/*
 * Dequeue one frame buffer from V4L2.
 * Returns v4l2 buffer index on success (>=0), -1 on error.
 * The corresponding MppBuffer is available via v4l2_capture_get_mpp_buffer().
 * Caller MUST call v4l2_capture_qbuf(idx) when done with the buffer.
 */
int  v4l2_capture_get_frame(V4L2Capture *c);

/*
 * Return the MppBuffer associated with a dequeued V4L2 buffer index.
 * Returns NULL if index is invalid.
 */
void* v4l2_capture_get_mpp_buffer(V4L2Capture *c, int idx);

/*
 * Return the mmap'd data pointer for a dequeued V4L2 buffer.
 */
void* v4l2_capture_get_data_ptr(V4L2Capture *c, int idx);

/*
 * Return data size (bytesused) for a dequeued V4L2 buffer.
 */
size_t v4l2_capture_get_data_size(V4L2Capture *c, int idx);

/*
 * Queue a buffer back to V4L2 for reuse.
 * Call after encoding is done with the buffer.
 */
int  v4l2_capture_qbuf(V4L2Capture *c, int idx);

/*
 * Stop streaming, unmap buffers, close device. Safe to call with NULL.
 */
void v4l2_capture_close(V4L2Capture *c);

#endif /* V4L2_CAPTURE_H */
