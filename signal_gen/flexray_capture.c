#include "flexray_capture.h"
#include "flexray_capture.pio.h"

#include <string.h>
#include "flexray_frame.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/structs/pio.h"
#include "pico/stdlib.h"
#include "tusb.h"

#define CAPTURE_NOTIFY_RING_SIZE 256u
#define CAPTURE_FSS_TIME_RING_SIZE 512u
#define CAPTURE_FSS_TIME_RING_MASK (CAPTURE_FSS_TIME_RING_SIZE - 1u)
#define CAPTURE_MAX_FRAME_BYTES (5u + MAX_FRAME_PAYLOAD_BYTES + 3u)

typedef struct {
    bool ready;
    uint sm;
    uint dma_chan;
    uint8_t source;
    volatile uint8_t ring[FLEXRAY_CAPTURE_RING_SIZE_BYTES] __attribute__((aligned(FLEXRAY_CAPTURE_RING_SIZE_BYTES)));
    volatile uint32_t irq_write_idx;
    uint16_t consumed_idx;
    uint32_t last_fss_us;
    uint64_t fss_delta_sum_us;
    volatile uint32_t fss_time_ring[CAPTURE_FSS_TIME_RING_SIZE];
    volatile uint16_t fss_time_head;
    volatile uint16_t fss_time_tail;
    uint16_t prev_valid_frame_id;
    uint32_t prev_valid_fss_us;
    bool have_prev_valid_frame;
} capture_channel_state_t;

typedef struct {
    uint8_t channel;
    uint16_t end_idx;
} capture_notification_t;

static PIO capture_pio;
static uint capture_program_offset;
static capture_channel_state_t capture_channels[FLEXRAY_CAPTURE_CHANNELS];
static uint capture_num_channels;

static volatile capture_notification_t notify_ring[CAPTURE_NOTIFY_RING_SIZE];
static volatile uint16_t notify_head;
static volatile uint16_t notify_tail;

static flexray_capture_diag_t capture_diag_state;
static uint8_t capture_segment[FLEXRAY_CAPTURE_RING_SIZE_BYTES];
static bool capture_usb_streaming = true;
static uint64_t pair_delta_sum_us;

static bool stream_frame(uint8_t source, const uint8_t *frame, uint16_t frame_len);

static inline uint32_t dma_ring_write_idx(uint dma_chan, volatile uint8_t *ring_base)
{
    uint32_t wa = dma_channel_hw_addr(dma_chan)->write_addr;
    return (wa - (uint32_t)(uintptr_t)ring_base) & FLEXRAY_CAPTURE_RING_MASK;
}

static inline bool pio_irq_is_set(PIO pio, uint irq_num)
{
    return (pio->irq & (1u << irq_num)) != 0;
}

static bool notify_push(uint8_t channel, uint16_t end_idx)
{
    uint16_t head = notify_head;
    uint16_t next = (uint16_t)((head + 1u) & (CAPTURE_NOTIFY_RING_SIZE - 1u));
    if (next == notify_tail) {
        capture_diag_state.dropped_notifications++;
        return false;
    }
    notify_ring[head] = (capture_notification_t){ .channel = channel, .end_idx = end_idx };
    notify_head = next;
    capture_diag_state.notifications++;
    return true;
}

static bool notify_pop(capture_notification_t *out)
{
    uint16_t tail = notify_tail;
    if (tail == notify_head) {
        return false;
    }
    *out = notify_ring[tail];
    notify_tail = (uint16_t)((tail + 1u) & (CAPTURE_NOTIFY_RING_SIZE - 1u));
    return true;
}

static void fss_time_push(capture_channel_state_t *state, uint32_t timestamp_us)
{
    uint16_t head = state->fss_time_head;
    uint16_t next = (uint16_t)((head + 1u) & CAPTURE_FSS_TIME_RING_MASK);
    if (next == state->fss_time_tail) {
        state->fss_time_tail = (uint16_t)((state->fss_time_tail + 1u) & CAPTURE_FSS_TIME_RING_MASK);
    }
    state->fss_time_ring[head] = timestamp_us;
    state->fss_time_head = next;
}

static bool fss_time_pop(capture_channel_state_t *state, uint32_t *timestamp_us)
{
    uint16_t tail = state->fss_time_tail;
    if (tail == state->fss_time_head) {
        return false;
    }
    *timestamp_us = state->fss_time_ring[tail];
    state->fss_time_tail = (uint16_t)((tail + 1u) & CAPTURE_FSS_TIME_RING_MASK);
    return true;
}

static uint16_t frame_id_from_header(const uint8_t *frame)
{
    return (uint16_t)(((uint16_t)(frame[0] & 0x07u) << 8) | frame[1]);
}

static void update_pair_measurement(capture_channel_state_t *state, uint8_t source,
                                    const uint8_t *frame, uint32_t fss_us)
{
    uint16_t frame_id = frame_id_from_header(frame);
    if (capture_diag_state.pair_enabled &&
        (capture_diag_state.pair_source == 0 || capture_diag_state.pair_source == source) &&
        state->have_prev_valid_frame &&
        state->prev_valid_frame_id == capture_diag_state.pair_from_id &&
        frame_id == capture_diag_state.pair_to_id) {
        uint32_t delta = fss_us - state->prev_valid_fss_us;
        capture_diag_state.pair_last_us = delta;
        if (capture_diag_state.pair_count == 0 || delta < capture_diag_state.pair_min_us) {
            capture_diag_state.pair_min_us = delta;
        }
        if (delta > capture_diag_state.pair_max_us) {
            capture_diag_state.pair_max_us = delta;
        }
        capture_diag_state.pair_count++;
        pair_delta_sum_us += delta;
        capture_diag_state.pair_avg_us =
            (uint32_t)(pair_delta_sum_us / capture_diag_state.pair_count);
    }

    state->prev_valid_frame_id = frame_id;
    state->prev_valid_fss_us = fss_us;
    state->have_prev_valid_frame = true;
}

static void record_fss_event(uint ch)
{
    if (ch >= capture_num_channels) {
        return;
    }
    capture_channel_state_t *state = &capture_channels[ch];
    if (!state->ready) {
        return;
    }

    uint32_t now = time_us_32();
    fss_time_push(state, now);
    if (state->last_fss_us != 0) {
        uint32_t delta = now - state->last_fss_us;
        capture_diag_state.fss_delta_last_us[ch] = delta;
        if (capture_diag_state.fss_delta_count[ch] == 0 ||
            delta < capture_diag_state.fss_delta_min_us[ch]) {
            capture_diag_state.fss_delta_min_us[ch] = delta;
        }
        if (delta > capture_diag_state.fss_delta_max_us[ch]) {
            capture_diag_state.fss_delta_max_us[ch] = delta;
        }
        capture_diag_state.fss_delta_count[ch]++;
        state->fss_delta_sum_us += delta;
        capture_diag_state.fss_delta_avg_us[ch] =
            (uint32_t)(state->fss_delta_sum_us / capture_diag_state.fss_delta_count[ch]);
    }
    state->last_fss_us = now;
}

static void __time_critical_func(capture_irq_handler)(void)
{
    for (uint ch = 0; ch < capture_num_channels; ch++) {
        uint irq_num = 4u + capture_channels[ch].sm;
        if (irq_num < 8u && pio_irq_is_set(capture_pio, irq_num)) {
            pio_interrupt_clear(capture_pio, irq_num);
            record_fss_event(ch);
        }
    }

    if (!pio_irq_is_set(capture_pio, 3)) {
        return;
    }
    pio_interrupt_clear(capture_pio, 3);

    for (uint ch = 0; ch < capture_num_channels; ch++) {
        capture_channel_state_t *state = &capture_channels[ch];
        if (!state->ready) {
            continue;
        }
        uint32_t idx_now = dma_ring_write_idx(state->dma_chan, state->ring);
        if (idx_now != state->irq_write_idx) {
            state->irq_write_idx = idx_now;
            (void)notify_push((uint8_t)ch, (uint16_t)idx_now);
        }
    }
}

static uint16_t header_crc_from_header(const uint8_t *header)
{
    return (uint16_t)(((uint16_t)(header[2] & 0x01) << 10) |
                      ((uint16_t)header[3] << 2) |
                      ((header[4] >> 6) & 0x03));
}

static bool frame_is_valid(const uint8_t *frame, uint16_t len)
{
    if (len < 8u) {
        return false;
    }
    uint8_t payload_words = (frame[2] >> 1) & 0x7Fu;
    uint16_t expected_len = (uint16_t)(5u + ((uint16_t)payload_words * 2u) + 3u);
    if (len != expected_len || expected_len > CAPTURE_MAX_FRAME_BYTES) {
        return false;
    }
    if (calculate_flexray_header_crc(frame) != header_crc_from_header(frame)) {
        return false;
    }
    uint32_t expected_crc = calculate_flexray_frame_crc(frame, (uint16_t)(expected_len - 3u));
    uint32_t actual_crc = ((uint32_t)frame[expected_len - 3u] << 16) |
                          ((uint32_t)frame[expected_len - 2u] << 8) |
                          frame[expected_len - 1u];
    return expected_crc == actual_crc;
}

static void stream_valid_frames_from_segment(capture_channel_state_t *state,
                                             const uint8_t *segment, uint16_t len)
{
    uint16_t pos = 0;

    while ((uint16_t)(len - pos) >= 8u) {
        const uint8_t *frame = segment + pos;
        uint8_t payload_words = (frame[2] >> 1) & 0x7Fu;
        uint16_t frame_len = (uint16_t)(5u + ((uint16_t)payload_words * 2u) + 3u);

        if (frame_len < 8u || frame_len > CAPTURE_MAX_FRAME_BYTES) {
            capture_diag_state.invalid_frames++;
            pos++;
            continue;
        }

        if ((uint16_t)(len - pos) < frame_len) {
            break;
        }

        if (frame_is_valid(frame, frame_len)) {
            uint32_t fss_us;
            if (fss_time_pop(state, &fss_us)) {
                update_pair_measurement(state, state->source, frame, fss_us);
            }
            (void)stream_frame(state->source, frame, frame_len);
            pos = (uint16_t)(pos + frame_len);
        } else {
            capture_diag_state.invalid_frames++;
            pos++;
        }
    }
}

static bool stream_frame(uint8_t source, const uint8_t *frame, uint16_t frame_len)
{
    if (!capture_usb_streaming) {
        capture_diag_state.frames_streamed++;
        return true;
    }

    uint16_t body_len = (uint16_t)(1u + frame_len);
    uint16_t total_len = (uint16_t)(2u + body_len);
    if (!tud_vendor_mounted() || tud_vendor_write_available() < total_len) {
        capture_diag_state.usb_backpressure++;
        return false;
    }

    uint8_t header[3] = {
        (uint8_t)(body_len & 0xFFu),
        (uint8_t)(body_len >> 8),
        source,
    };
    if (tud_vendor_write(header, sizeof(header)) != sizeof(header)) {
        capture_diag_state.usb_backpressure++;
        return false;
    }
    if (tud_vendor_write(frame, frame_len) != frame_len) {
        capture_diag_state.usb_backpressure++;
        return false;
    }
    tud_vendor_write_flush();
    capture_diag_state.frames_streamed++;
    return true;
}

bool flexray_capture_init(PIO pio, const fr_capture_channel_t *channels, uint num_channels)
{
    if (channels == NULL || num_channels > FLEXRAY_CAPTURE_CHANNELS) {
        return false;
    }

    capture_pio = pio;
    capture_num_channels = num_channels;
    notify_head = 0;
    notify_tail = 0;
    memset(&capture_diag_state, 0, sizeof(capture_diag_state));
    memset(capture_channels, 0, sizeof(capture_channels));

    capture_program_offset = pio_add_program(pio, &flexray_capture_program);

    for (uint ch = 0; ch < num_channels; ch++) {
        capture_channel_state_t *state = &capture_channels[ch];
        state->sm = pio_claim_unused_sm(pio, true);
        state->source = channels[ch].source;
        state->dma_chan = dma_claim_unused_channel(true);
        state->irq_write_idx = 0;
        state->consumed_idx = 0;

        flexray_capture_program_init(pio, state->sm, capture_program_offset, channels[ch].rx_pin);

        dma_channel_config cfg = dma_channel_get_default_config(state->dma_chan);
        channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
        channel_config_set_read_increment(&cfg, false);
        channel_config_set_write_increment(&cfg, true);
        channel_config_set_dreq(&cfg, pio_get_dreq(pio, state->sm, false));
        channel_config_set_ring(&cfg, true, 12);
        dma_channel_configure(state->dma_chan, &cfg,
                              (void *)state->ring,
                              &pio->rxf[state->sm],
                              (uint32_t)(FLEXRAY_CAPTURE_RING_SIZE_BYTES | 0x10000000u),
                              true);

        pio_sm_clear_fifos(pio, state->sm);
        pio_sm_set_enabled(pio, state->sm, true);
        state->ready = true;
    }

    pio_set_irq0_source_enabled(pio, pis_interrupt3, true);
#if PICO_PIO_VERSION > 0
    pio_set_irq0_source_enabled(pio, pis_interrupt4, true);
    pio_set_irq0_source_enabled(pio, pis_interrupt5, true);
#endif
    irq_set_exclusive_handler(pio_get_irq_num(pio, 0), capture_irq_handler);
    irq_set_enabled(pio_get_irq_num(pio, 0), true);
    pio_interrupt_clear(pio, 3);
    pio_interrupt_clear(pio, 4);
    pio_interrupt_clear(pio, 5);
    pio_interrupt_clear(pio, 7);
    return true;
}

void flexray_capture_task(void)
{
    capture_notification_t note;

    while (notify_pop(&note)) {
        if (note.channel >= capture_num_channels) {
            continue;
        }
        capture_channel_state_t *state = &capture_channels[note.channel];
        uint16_t len = (uint16_t)((note.end_idx - state->consumed_idx) & FLEXRAY_CAPTURE_RING_MASK);
        state->consumed_idx = note.end_idx;
        if (len == 0u) {
            capture_diag_state.invalid_frames++;
            continue;
        }

        uint16_t start = (uint16_t)((note.end_idx - len) & FLEXRAY_CAPTURE_RING_MASK);
        uint16_t first = (uint16_t)((len <= (FLEXRAY_CAPTURE_RING_SIZE_BYTES - start))
                                      ? len
                                      : (FLEXRAY_CAPTURE_RING_SIZE_BYTES - start));
        memcpy(capture_segment, (const void *)(state->ring + start), first);
        if (first < len) {
            memcpy(capture_segment + first, (const void *)state->ring, (size_t)(len - first));
        }
        stream_valid_frames_from_segment(state, capture_segment, len);
    }
}

void flexray_capture_diag(flexray_capture_diag_t *diag)
{
    if (diag != NULL) {
        *diag = capture_diag_state;
    }
}

void flexray_capture_reset_timing(void)
{
    for (uint ch = 0; ch < FLEXRAY_CAPTURE_CHANNELS; ch++) {
        capture_channels[ch].last_fss_us = 0;
        capture_channels[ch].fss_delta_sum_us = 0;
        capture_channels[ch].fss_time_head = 0;
        capture_channels[ch].fss_time_tail = 0;
        capture_channels[ch].have_prev_valid_frame = false;
        capture_diag_state.fss_delta_last_us[ch] = 0;
        capture_diag_state.fss_delta_min_us[ch] = 0;
        capture_diag_state.fss_delta_max_us[ch] = 0;
        capture_diag_state.fss_delta_avg_us[ch] = 0;
        capture_diag_state.fss_delta_count[ch] = 0;
    }
    capture_diag_state.pair_last_us = 0;
    capture_diag_state.pair_min_us = 0;
    capture_diag_state.pair_max_us = 0;
    capture_diag_state.pair_avg_us = 0;
    capture_diag_state.pair_count = 0;
    pair_delta_sum_us = 0;
}

void flexray_capture_set_usb_streaming(bool enabled)
{
    capture_usb_streaming = enabled;
}

void flexray_capture_set_timing_pair(uint8_t source, uint16_t from_id, uint16_t to_id)
{
    capture_diag_state.pair_source = source;
    capture_diag_state.pair_from_id = from_id;
    capture_diag_state.pair_to_id = to_id;
    capture_diag_state.pair_enabled = true;
    flexray_capture_reset_timing();
}
