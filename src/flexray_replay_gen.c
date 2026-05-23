#include "flexray_replay_gen.h"
#include "flexray_replay_gen.pio.h"

#include <stdio.h>
#include <string.h>
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/regs/dma.h"
#include "hardware/structs/pio.h"
#include "pico/time.h"

#define REPLAY_BUFFER_COUNT 2u
#define FLEXRAY_BITS_PER_US 10u
#define INTERLEAVED_BITS_PER_WIRE_BIT 2u
#define CYCLE_BITS (FLEXRAY_REPLAY_CYCLE_PERIOD_US * FLEXRAY_BITS_PER_US)
#define CYCLE_STREAM_BITS (CYCLE_BITS * INTERLEAVED_BITS_PER_WIRE_BIT)
#define CYCLE_BYTES (((CYCLE_STREAM_BITS + 7u) / 8u + 3u) & ~3u)
#define CYCLE_WORDS (CYCLE_BYTES / 4u)
#define TX_FIFO_FILL_TIMEOUT_US 1000u

typedef struct {
    bool active;
    uint8_t cycle_base;
    uint8_t cycle_mask;
    uint16_t frame_id;
    uint8_t indicators;
    uint16_t payload_len;
    uint8_t payload[MAX_FRAME_PAYLOAD_BYTES];
    uint8_t frame_buf[MAX_FRAME_BUF_SIZE_BYTES] __attribute__((aligned(4)));
} replay_slot_t;

typedef struct {
    uint8_t *buf;
    uint32_t bit_pos;
    uint32_t bit_limit;
} bit_writer_t;

static PIO replay_pio;
static uint replay_sm;
static uint replay_program_offset;
static uint replay_tx_pin;
static uint replay_txen_pin;
static int replay_dma_chan[REPLAY_BUFFER_COUNT];
static dma_channel_config replay_dma_cfg[REPLAY_BUFFER_COUNT];
static replay_slot_t replay_slots[FLEXRAY_REPLAY_MAX_SLOTS];
static uint8_t replay_cycle_bufs[REPLAY_BUFFER_COUNT][CYCLE_BYTES] __attribute__((aligned(4)));

static bool replay_hw_ready;
static bool replay_running;
static uint32_t replay_slot_duration_us;
static volatile uint32_t dma_completed_cycles;
static uint32_t handled_completed_cycles;
static uint32_t tx_count;
static uint32_t late_buffer_count;
static uint32_t txstall_count;
static uint32_t last_render_us;
static uint32_t max_render_us;

static inline void bw_append_bit(bit_writer_t *bw, bool bit)
{
    if (bw->bit_pos >= bw->bit_limit) {
        return;
    }
    if (!bit) {
        bw->buf[bw->bit_pos >> 3] &= (uint8_t)~(0x80u >> (bw->bit_pos & 7u));
    }
    bw->bit_pos++;
}

static inline void bw_append_wire_bit(bit_writer_t *bw, bool txd, bool txen)
{
    bw_append_bit(bw, txen);
    bw_append_bit(bw, txd);
}

static void bw_append_byte(bit_writer_t *bw, uint8_t byte)
{
    for (int bit = 7; bit >= 0; bit--) {
        bw_append_wire_bit(bw, (byte >> bit) & 1u, false);
    }
}

static void bw_skip_to(bit_writer_t *bw, uint32_t bit_pos)
{
    if (bit_pos > bw->bit_limit) {
        bit_pos = bw->bit_limit;
    }
    if (bw->bit_pos < bit_pos) {
        bw->bit_pos = bit_pos;
    }
}

static uint32_t calc_payload_slot_duration(uint16_t payload_len)
{
    uint32_t frame_bytes = (uint32_t)payload_len + 8u;
    uint32_t wire_us = frame_bytes + 3u;
    uint32_t dur = wire_us + 10u;
    dur = ((dur + 4u) / 5u) * 5u;
    if (dur < 25u) {
        dur = 25u;
    }
    return dur;
}

static uint32_t calc_slot_duration(void)
{
    uint16_t max_payload = 0;
    for (uint s = 0; s < FLEXRAY_REPLAY_MAX_SLOTS; s++) {
        if (replay_slots[s].active && replay_slots[s].payload_len > max_payload) {
            max_payload = replay_slots[s].payload_len;
        }
    }
    return calc_payload_slot_duration(max_payload);
}

static uint32_t active_slot_extent(void)
{
    uint32_t extent = 0;
    for (uint s = 0; s < FLEXRAY_REPLAY_MAX_SLOTS; s++) {
        if (replay_slots[s].active) {
            extent = s + 1u;
        }
    }
    return extent;
}

static void build_header(uint8_t *buf, uint16_t frame_id, uint8_t indicators,
                         uint8_t payload_length_words, uint8_t cycle_count)
{
    buf[0] = (uint8_t)((indicators << 3) | ((frame_id >> 8) & 0x07));
    buf[1] = (uint8_t)(frame_id & 0xFF);
    buf[2] = (uint8_t)(payload_length_words << 1);
    uint16_t hcrc = calculate_flexray_header_crc(buf);
    buf[2] = (uint8_t)((payload_length_words << 1) | ((hcrc >> 10) & 0x01));
    buf[3] = (uint8_t)((hcrc >> 2) & 0xFF);
    buf[4] = (uint8_t)(((hcrc & 0x03) << 6) | (cycle_count & 0x3F));
}

static uint32_t build_frame(replay_slot_t *slot, uint8_t cycle_count)
{
    uint8_t payload_len_words = (uint8_t)(slot->payload_len / 2u);
    build_header(slot->frame_buf, slot->frame_id, slot->indicators,
                 payload_len_words, cycle_count);
    if (slot->payload_len > 0) {
        memcpy(slot->frame_buf + 5, slot->payload, slot->payload_len);
    }

    uint16_t before_crc = (uint16_t)(5u + slot->payload_len);
    uint32_t crc = calculate_flexray_frame_crc(slot->frame_buf, before_crc);
    slot->frame_buf[before_crc + 0u] = (uint8_t)(crc >> 16);
    slot->frame_buf[before_crc + 1u] = (uint8_t)(crc >> 8);
    slot->frame_buf[before_crc + 2u] = (uint8_t)crc;
    return (uint32_t)before_crc + 3u;
}

static void render_frame_bits(replay_slot_t *slot, uint32_t frame_byte_count,
                              bit_writer_t *bw, uint32_t slot_end_bit)
{
    for (uint i = 0; i < 8u; i++) {
        bw_append_wire_bit(bw, false, false);
    }
    bw_append_wire_bit(bw, true, false);

    for (uint32_t i = 0; i < frame_byte_count; i++) {
        bw_append_wire_bit(bw, true, false);
        bw_append_wire_bit(bw, false, false);
        bw_append_byte(bw, slot->frame_buf[i]);
    }

    bw_append_wire_bit(bw, false, false);
    for (uint i = 0; i < 11u; i++) {
        bw_append_wire_bit(bw, true, false);
    }
    bw_skip_to(bw, slot_end_bit);
}

static void render_cycle_to_buffer(uint8_t cycle_count, uint bank)
{
    absolute_time_t start = get_absolute_time();
    uint32_t slot_bits = replay_slot_duration_us * FLEXRAY_BITS_PER_US *
                         INTERLEAVED_BITS_PER_WIRE_BIT;
    bit_writer_t bw = {
        .buf = replay_cycle_bufs[bank],
        .bit_pos = 0,
        .bit_limit = CYCLE_STREAM_BITS,
    };

    memset(replay_cycle_bufs[bank], 0xFF, sizeof(replay_cycle_bufs[bank]));
    for (uint s = 0; s < FLEXRAY_REPLAY_MAX_SLOTS; s++) {
        replay_slot_t *slot = &replay_slots[s];
        uint32_t slot_end_bit = (s + 1u) * slot_bits;
        if (slot_end_bit > CYCLE_STREAM_BITS) {
            slot_end_bit = CYCLE_STREAM_BITS;
        }

        if (slot->active &&
            (uint8_t)(cycle_count & slot->cycle_mask) == slot->cycle_base) {
            uint32_t frame_byte_count = build_frame(slot, cycle_count);
            render_frame_bits(slot, frame_byte_count, &bw, slot_end_bit);
        } else {
            bw_skip_to(&bw, slot_end_bit);
        }
    }

    last_render_us = (uint32_t)absolute_time_diff_us(start, get_absolute_time());
    if (last_render_us > max_render_us) {
        max_render_us = last_render_us;
    }
}

static void stop_dma_chan(int dma_chan)
{
    uint chan = (uint)dma_chan;
    dma_irqn_set_channel_enabled(0, chan, false);
    dma_irqn_acknowledge_channel(0, chan);
    dma_channel_hw_addr(chan)->ctrl_trig &= ~DMA_CH0_CTRL_TRIG_EN_BITS;
    dma_channel_abort(chan);
    dma_irqn_acknowledge_channel(0, chan);
}

static void park_replay_sm(void)
{
    pio_sm_set_enabled(replay_pio, replay_sm, false);
    pio_sm_set_pins_with_mask(replay_pio, replay_sm,
                              (1u << replay_tx_pin) | (1u << replay_txen_pin),
                              (1u << replay_tx_pin) | (1u << replay_txen_pin));
}

static void configure_dma_bank(uint bank)
{
    uint dma_chan = (uint)replay_dma_chan[bank];
    dma_channel_set_config(dma_chan, &replay_dma_cfg[bank], false);
    dma_channel_set_write_addr(dma_chan, (void *)&replay_pio->txf[replay_sm], false);
    dma_channel_set_read_addr(dma_chan, replay_cycle_bufs[bank], false);
    dma_channel_set_transfer_count(dma_chan, CYCLE_WORDS, false);
}

static void dma_cycle_irq_handler(void)
{
    if (!replay_hw_ready) {
        return;
    }
    for (uint bank = 0; bank < REPLAY_BUFFER_COUNT; bank++) {
        uint dma_chan = (uint)replay_dma_chan[bank];
        if (dma_irqn_get_channel_status(0, dma_chan)) {
            dma_irqn_acknowledge_channel(0, dma_chan);
            dma_completed_cycles++;
        }
    }
}

static void ensure_dma_cycle_irq(void)
{
    static bool installed;
    if (installed) {
        return;
    }
    irq_set_exclusive_handler(DMA_IRQ_NUM(0), dma_cycle_irq_handler);
    irq_set_priority(DMA_IRQ_NUM(0), 0);
    irq_set_enabled(DMA_IRQ_NUM(0), true);
    installed = true;
}

bool flexray_replay_init(PIO pio, uint tx_pin, uint txen_pin)
{
    replay_pio = pio;
    replay_tx_pin = tx_pin;
    replay_txen_pin = txen_pin;
    replay_running = false;
    memset(replay_slots, 0, sizeof(replay_slots));
    memset(replay_cycle_bufs, 0xFF, sizeof(replay_cycle_bufs));

    replay_program_offset = pio_add_program(pio, &flexray_replay_gen_program);
    replay_sm = pio_claim_unused_sm(pio, true);
    flexray_replay_gen_program_init(pio, replay_sm, replay_program_offset,
                                    tx_pin, txen_pin);

    for (uint bank = 0; bank < REPLAY_BUFFER_COUNT; bank++) {
        replay_dma_chan[bank] = (int)dma_claim_unused_channel(true);
        replay_dma_cfg[bank] = dma_channel_get_default_config((uint)replay_dma_chan[bank]);
        channel_config_set_transfer_data_size(&replay_dma_cfg[bank], DMA_SIZE_32);
        channel_config_set_bswap(&replay_dma_cfg[bank], true);
        channel_config_set_read_increment(&replay_dma_cfg[bank], true);
        channel_config_set_write_increment(&replay_dma_cfg[bank], false);
        channel_config_set_dreq(&replay_dma_cfg[bank], pio_get_dreq(pio, replay_sm, true));
        channel_config_set_high_priority(&replay_dma_cfg[bank], true);
    }
    channel_config_set_chain_to(&replay_dma_cfg[0], (uint)replay_dma_chan[1]);
    channel_config_set_chain_to(&replay_dma_cfg[1], (uint)replay_dma_chan[0]);

    for (uint bank = 0; bank < REPLAY_BUFFER_COUNT; bank++) {
        dma_channel_set_config((uint)replay_dma_chan[bank], &replay_dma_cfg[bank], false);
        dma_channel_set_write_addr((uint)replay_dma_chan[bank],
                                   (void *)&pio->txf[replay_sm], false);
    }

    replay_hw_ready = true;
    printf("Replay gen: sm=%u dma0=%d dma1=%d tx=%u txen=%u\n",
           replay_sm, replay_dma_chan[0], replay_dma_chan[1], tx_pin, txen_pin);
    return true;
}

bool flexray_replay_set_slot(uint slot, uint8_t cycle_base, uint8_t cycle_mask,
                             uint16_t frame_id, uint8_t indicators,
                             const uint8_t *payload, uint16_t payload_len)
{
    if (slot >= FLEXRAY_REPLAY_MAX_SLOTS) {
        return false;
    }
    if (frame_id > 2047u || payload_len > MAX_FRAME_PAYLOAD_BYTES || (payload_len & 1u)) {
        return false;
    }
    if (payload_len > 0 && payload == NULL) {
        return false;
    }
    if (replay_running && calc_payload_slot_duration(payload_len) > replay_slot_duration_us) {
        return false;
    }

    replay_slot_t *sl = &replay_slots[slot];
    sl->cycle_base = cycle_base & cycle_mask;
    sl->cycle_mask = cycle_mask;
    sl->frame_id = frame_id;
    sl->indicators = indicators & 0x1Fu;
    sl->payload_len = payload_len;
    if (payload_len > 0) {
        memcpy(sl->payload, payload, payload_len);
    }
    sl->active = true;
    return true;
}

void flexray_replay_clear_slot(uint slot)
{
    if (slot >= FLEXRAY_REPLAY_MAX_SLOTS) {
        return;
    }
    replay_slots[slot].active = false;
}

void flexray_replay_clear_all_slots(void)
{
    for (uint slot = 0; slot < FLEXRAY_REPLAY_MAX_SLOTS; slot++) {
        replay_slots[slot].active = false;
    }
}

bool flexray_replay_can_start(void)
{
    uint32_t extent = active_slot_extent();
    uint32_t duration = calc_slot_duration();
    if (extent == 0 || (extent * duration) > FLEXRAY_REPLAY_CYCLE_PERIOD_US) {
        printf("Replay start rejected: slots=%lu gdStaticSlot=%lu us\n",
               (unsigned long)extent, (unsigned long)duration);
        return false;
    }
    return true;
}

bool flexray_replay_start(void)
{
    if (!replay_hw_ready || !flexray_replay_can_start()) {
        return false;
    }

    for (uint bank = 0; bank < REPLAY_BUFFER_COUNT; bank++) {
        stop_dma_chan(replay_dma_chan[bank]);
    }
    park_replay_sm();
    flexray_replay_gen_program_init(replay_pio, replay_sm, replay_program_offset,
                                    replay_tx_pin, replay_txen_pin);
    pio_sm_clear_fifos(replay_pio, replay_sm);

    replay_slot_duration_us = calc_slot_duration();
    dma_completed_cycles = 0;
    handled_completed_cycles = 0;
    tx_count = 0;
    late_buffer_count = 0;
    txstall_count = 0;
    last_render_us = 0;
    max_render_us = 0;

    render_cycle_to_buffer(0, 0);
    render_cycle_to_buffer(1, 1);
    configure_dma_bank(0);
    configure_dma_bank(1);
    ensure_dma_cycle_irq();

    pio_sm_clear_fifos(replay_pio, replay_sm);
    pio_sm_restart(replay_pio, replay_sm);
    pio_sm_exec(replay_pio, replay_sm, pio_encode_jmp(replay_program_offset));
    replay_pio->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + replay_sm);

    for (uint bank = 0; bank < REPLAY_BUFFER_COUNT; bank++) {
        uint dma_chan = (uint)replay_dma_chan[bank];
        dma_irqn_acknowledge_channel(0, dma_chan);
        dma_irqn_set_channel_enabled(0, dma_chan, true);
    }

    dma_start_channel_mask(1u << (uint)replay_dma_chan[0]);
    absolute_time_t deadline = make_timeout_time_us(TX_FIFO_FILL_TIMEOUT_US);
    while (!pio_sm_is_tx_fifo_full(replay_pio, replay_sm) &&
           absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
        tight_loop_contents();
    }
    pio_sm_set_enabled(replay_pio, replay_sm, true);

    replay_running = true;
    printf("Replay start: slots=%lu gdStaticSlot=%lu us\n",
           (unsigned long)active_slot_extent(),
           (unsigned long)replay_slot_duration_us);
    return true;
}

void flexray_replay_stop(void)
{
    if (!replay_hw_ready) {
        return;
    }
    replay_running = false;
    for (uint bank = 0; bank < REPLAY_BUFFER_COUNT; bank++) {
        stop_dma_chan(replay_dma_chan[bank]);
    }
    park_replay_sm();
    pio_sm_clear_fifos(replay_pio, replay_sm);
    printf("Replay stop after %lu cycles\n", (unsigned long)tx_count);
}

bool flexray_replay_is_running(void)
{
    return replay_running;
}

void flexray_replay_task(void)
{
    if (!replay_running) {
        return;
    }

    uint32_t completed = dma_completed_cycles;
    if (completed > tx_count) {
        tx_count = completed;
    }

    uint32_t lag = completed - handled_completed_cycles;
    if (lag > 1u) {
        late_buffer_count += lag - 1u;
    }

    while (handled_completed_cycles < completed) {
        uint completed_bank = handled_completed_cycles & 1u;
        uint8_t render_cycle = (uint8_t)((handled_completed_cycles + 2u) & 0x3Fu);
        render_cycle_to_buffer(render_cycle, completed_bank);
        configure_dma_bank(completed_bank);
        handled_completed_cycles++;
    }

    uint32_t stall_bit = 1u << (PIO_FDEBUG_TXSTALL_LSB + replay_sm);
    if (replay_pio->fdebug & stall_bit) {
        txstall_count++;
        replay_pio->fdebug = stall_bit;
    }
}

void flexray_replay_diag(flexray_replay_diag_t *diag)
{
    if (diag == NULL) {
        return;
    }
    diag->completed_cycles = dma_completed_cycles;
    diag->handled_cycles = handled_completed_cycles;
    diag->late_buffer_count = late_buffer_count;
    diag->txstall_count = txstall_count;
    diag->last_render_us = last_render_us;
    diag->max_render_us = max_render_us;
}
