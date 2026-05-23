#ifndef FLEXRAY_REPLAY_GEN_H
#define FLEXRAY_REPLAY_GEN_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware/pio.h"
#include "flexray_frame.h"

#define FLEXRAY_REPLAY_MAX_SLOTS 96u
#define FLEXRAY_REPLAY_CYCLE_PERIOD_US 5000u

typedef struct {
    uint32_t completed_cycles;
    uint32_t handled_cycles;
    uint32_t late_buffer_count;
    uint32_t txstall_count;
    uint32_t last_render_us;
    uint32_t max_render_us;
} flexray_replay_diag_t;

bool flexray_replay_init(PIO pio, uint tx_pin, uint txen_pin);
bool flexray_replay_set_slot(uint slot,
                             uint8_t cycle_base,
                             uint8_t cycle_mask,
                             uint16_t frame_id,
                             uint8_t indicators,
                             const uint8_t *payload,
                             uint16_t payload_len);
void flexray_replay_clear_slot(uint slot);
void flexray_replay_clear_all_slots(void);
bool flexray_replay_can_start(void);
bool flexray_replay_start(void);
void flexray_replay_stop(void);
bool flexray_replay_is_running(void);
void flexray_replay_task(void);
void flexray_replay_diag(flexray_replay_diag_t *diag);

#endif
