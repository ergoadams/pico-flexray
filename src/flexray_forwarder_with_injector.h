#ifndef FLEXRAY_FORWARDER_WITH_INJECTOR_H
#define FLEXRAY_FORWARDER_WITH_INJECTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/pio.h"

#define FLEXRAY_FILTER_DIR_FR1_TO_FR2 (1u << 0)
#define FLEXRAY_FILTER_DIR_FR2_TO_FR1 (1u << 1)
#define FLEXRAY_FILTER_DIR_FR3_TO_FR4 (1u << 2)
#define FLEXRAY_FILTER_DIR_FR4_TO_FR3 (1u << 3)

// Cache a frame's raw bytes (header+payload+CRC) when rules match
void try_cache_last_target_frame(uint16_t frame_id, uint8_t cycle_count, uint16_t frame_length, uint8_t *captured_bytes);

// On receiving a frame, check triggers; if matched, mutate template and request injection
void try_inject_frame(uint16_t frame_id, uint8_t cycle_count);

void setup_forwarder_with_injector(PIO pio,
    uint rx_pin_from_fr1, uint tx_pin_to_fr2,
    uint rx_pin_from_fr2, uint tx_pin_to_fr1,
    uint rx_pin_from_fr3, uint tx_pin_to_fr4,
    uint rx_pin_from_fr4, uint tx_pin_to_fr3);

// Submit a host-provided replacement slice to be used on next matching injection
// bytes must contain only the replacement payload slice; length must equal rule->replace_len
// The override applies when id matches a rule's target_id and (cycle_count & rule->cycle_mask) == rule->cycle_base
bool injector_submit_override(uint16_t id, uint8_t base, uint16_t len, const uint8_t *bytes);

// Enable/disable injection at runtime
void injector_set_enabled(bool enabled);
bool injector_is_enabled(void);

// Runtime PIO-assisted forwarding filter. direction_mask uses FLEXRAY_FILTER_DIR_* bits.
bool flexray_filter_set(uint8_t count, const uint16_t *ids, const uint8_t *direction_masks);
void flexray_filter_clear(void);
void flexray_filter_set_enabled(bool enabled);
bool flexray_filter_is_enabled(void);
bool flexray_filter_should_block(uint16_t frame_id, uint8_t direction);

// Called from streamer IRQs after an early header match and again at frame end.
void flexray_forwarder_suppress_source(uint8_t source);
void flexray_forwarder_release_source(uint8_t source);

#endif // FLEXRAY_FORWARDER_WITH_INJECTOR_H


