#ifndef FLEXRAY_CAPTURE_H
#define FLEXRAY_CAPTURE_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware/pio.h"

#define FLEXRAY_CAPTURE_CHANNELS 4u
#define FLEXRAY_CAPTURE_RING_SIZE_BYTES (1u << 12)
#define FLEXRAY_CAPTURE_RING_MASK (FLEXRAY_CAPTURE_RING_SIZE_BYTES - 1u)

typedef struct {
    uint rx_pin;
    uint8_t source;
} fr_capture_channel_t;

typedef struct {
    uint32_t notifications;
    uint32_t dropped_notifications;
    uint32_t frames_streamed;
    uint32_t invalid_frames;
    uint32_t usb_backpressure;
    uint32_t fss_delta_last_us[FLEXRAY_CAPTURE_CHANNELS];
    uint32_t fss_delta_min_us[FLEXRAY_CAPTURE_CHANNELS];
    uint32_t fss_delta_max_us[FLEXRAY_CAPTURE_CHANNELS];
    uint32_t fss_delta_avg_us[FLEXRAY_CAPTURE_CHANNELS];
    uint32_t fss_delta_count[FLEXRAY_CAPTURE_CHANNELS];
    uint8_t pair_enabled;
    uint8_t pair_source;
    uint16_t pair_from_id;
    uint16_t pair_to_id;
    uint32_t pair_last_us;
    uint32_t pair_min_us;
    uint32_t pair_max_us;
    uint32_t pair_avg_us;
    uint32_t pair_count;
} flexray_capture_diag_t;

bool flexray_capture_init(PIO pio, const fr_capture_channel_t *channels, uint num_channels);
void flexray_capture_task(void);
void flexray_capture_diag(flexray_capture_diag_t *diag);
void flexray_capture_reset_timing(void);
void flexray_capture_set_usb_streaming(bool enabled);
void flexray_capture_set_timing_pair(uint8_t source, uint16_t from_id, uint16_t to_id);

#endif
