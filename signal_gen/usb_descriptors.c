#include "tusb.h"
#include "pico/unique_id.h"
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/watchdog.h"
#include <string.h>

#define SIGNAL_GEN_VID 0x3801
#define SIGNAL_GEN_PID 0xddcc

#define PANDA_GET_MICROSECOND_TIMER     0xa8
#define PANDA_RESET_CAN_COMMS           0xc0
#define PANDA_GET_HW_TYPE               0xc1
#define PANDA_GET_CAN_HEALTH_STATS      0xc2
#define PANDA_GET_MCU_UID               0xc3
#define PANDA_FETCH_SERIAL_NUMBER       0xd0
#define PANDA_ENTER_BOOTLOADER_MODE     0xd1
#define PANDA_GET_HEALTH_PACKET         0xd2
#define PANDA_GET_SIGNATURE_PART1       0xd3
#define PANDA_GET_SIGNATURE_PART2       0xd4
#define PANDA_GET_GIT_VERSION           0xd6
#define PANDA_SYSTEM_RESET              0xd8
#define PANDA_SET_OBD_CAN_MUX_MODE      0xdb
#define PANDA_SET_SAFETY_MODEL          0xdc
#define PANDA_GET_VERSIONS              0xdd
#define PANDA_SET_CAN_SPEED_KBPS        0xde
#define PANDA_SET_ALT_EXPERIENCE        0xdf
#define PANDA_UART_READ                 0xe0
#define PANDA_SET_CAN_LOOPBACK          0xe5
#define PANDA_SET_CLOCK_SOURCE_PARAMS   0xe6
#define PANDA_SET_POWER_SAVE_STATE      0xe7
#define PANDA_SET_CAN_FD_AUTO_SWITCH    0xe8
#define PANDA_CAN_CLEAR_BUFFER          0xf1
#define PANDA_HEARTBEAT                 0xf3
#define PANDA_DISABLE_HEARTBEAT_CHECKS  0xf8
#define PANDA_SET_CAN_FD_DATA_BITRATE   0xf9

#define HW_TYPE_RED_PANDA               7
#define SAFETY_SILENT                   0

struct __attribute__((packed)) health_t {
    uint32_t uptime_pkt;
    uint32_t voltage_pkt;
    uint32_t current_pkt;
    uint32_t safety_tx_blocked_pkt;
    uint32_t safety_rx_invalid_pkt;
    uint32_t tx_buffer_overflow_pkt;
    uint32_t rx_buffer_overflow_pkt;
    uint32_t faults_pkt;
    uint8_t ignition_line_pkt;
    uint8_t ignition_can_pkt;
    uint8_t controls_allowed_pkt;
    uint8_t car_harness_status_pkt;
    uint8_t safety_mode_pkt;
    uint16_t safety_param_pkt;
    uint8_t fault_status_pkt;
    uint8_t power_save_enabled_pkt;
    uint8_t heartbeat_lost_pkt;
    uint16_t alternative_experience_pkt;
    float interrupt_load_pkt;
    uint8_t fan_power;
    uint8_t safety_rx_checks_invalid_pkt;
    uint16_t spi_error_count_pkt;
    uint16_t sbu1_voltage_mV;
    uint16_t sbu2_voltage_mV;
    uint8_t som_reset_triggered;
};

struct __attribute__((packed)) can_health_t {
    uint8_t bus_off;
    uint32_t bus_off_cnt;
    uint8_t error_warning;
    uint8_t error_passive;
    uint8_t last_error;
    uint8_t last_stored_error;
    uint8_t last_data_error;
    uint8_t last_data_stored_error;
    uint8_t receive_error_cnt;
    uint8_t transmit_error_cnt;
    uint32_t total_error_cnt;
    uint32_t total_tx_lost_cnt;
    uint32_t total_rx_lost_cnt;
    uint32_t total_tx_cnt;
    uint32_t total_rx_cnt;
    uint32_t total_fwd_cnt;
    uint32_t total_tx_checksum_error_cnt;
    uint16_t can_speed;
    uint16_t can_data_speed;
    uint8_t canfd_enabled;
    uint8_t brs_enabled;
    uint8_t canfd_non_iso;
    uint32_t irq0_call_rate;
    uint32_t irq1_call_rate;
    uint32_t irq2_call_rate;
    uint32_t can_core_reset_cnt;
};

static bool pending_reset;
static bool pending_bootloader;
static uint8_t safety_model = SAFETY_SILENT;
static uint16_t safety_param;
static uint16_t alternative_experience;

#if PICO_RP2040
#define SIGNAL_GEN_PRODUCT "FlexRay Signal Generator RP2040"
#elif PICO_RP2350
#define SIGNAL_GEN_PRODUCT "FlexRay Signal Generator RP2350"
#else
#define SIGNAL_GEN_PRODUCT "FlexRay Signal Generator"
#endif

#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN)

enum {
    ITF_NUM_VENDOR,
    ITF_NUM_TOTAL
};

enum {
    EPNUM_VENDOR_OUT = 0x03,
    EPNUM_VENDOR_IN  = 0x81
};

// ---------- Device Descriptor ----------
tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = SIGNAL_GEN_VID,
    .idProduct          = SIGNAL_GEN_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

// ---------- Configuration Descriptor ----------
uint8_t const desc_cfg[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN, 0x00, 100),
    TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, 4, EPNUM_VENDOR_OUT, EPNUM_VENDOR_IN, 64)
};

// ---------- String Descriptors ----------
enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_INTERFACE,
};

char const *string_desc_arr[] = {
    (char[]){0x09, 0x04},       // 0: English
    "comma.ai",                 // 1: Manufacturer
    SIGNAL_GEN_PRODUCT,         // 2: Product
    NULL,                       // 3: Serial (filled at runtime)
    "panda"                     // 4: Interface
};

static uint16_t _desc_str[32];
static char serial_str[25];

#define FLEXRAY_SERIAL_PREFIX "picoflex"

// ---------- TinyUSB Callbacks ----------
uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&desc_device;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_cfg;
}

uint8_t const *tud_descriptor_bos_cb(void) {
    return NULL;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    uint8_t chr_count;

    if (index == 0) {
        memcpy(&_desc_str[0], string_desc_arr[0], 2);
        chr_count = 1;
    } else if (index == 3) {
        pico_unique_board_id_t board_id;
        pico_get_unique_board_id(&board_id);
        static const char hex[] = "0123456789abcdef";
        memcpy(serial_str, FLEXRAY_SERIAL_PREFIX, strlen(FLEXRAY_SERIAL_PREFIX));
        uint8_t pos = (uint8_t)strlen(FLEXRAY_SERIAL_PREFIX);
        for (int i = 0; i < 8; i++) {
            serial_str[pos++] = hex[board_id.id[i] >> 4];
            serial_str[pos++] = hex[board_id.id[i] & 0x0F];
        }
        serial_str[pos] = '\0';
        string_desc_arr[3] = serial_str;
        chr_count = (uint8_t)strlen(serial_str);
        if (chr_count > 31) chr_count = 31;
        for (uint8_t i = 0; i < chr_count; i++)
            _desc_str[1 + i] = (uint16_t)serial_str[i];
    } else if (index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) {
        const char *str = string_desc_arr[index];
        chr_count = (uint8_t)strlen(str);
        if (chr_count > 31) chr_count = 31;
        for (uint8_t i = 0; i < chr_count; i++)
            _desc_str[1 + i] = (uint16_t)str[i];
    } else {
        return NULL;
    }
    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}

static void make_serial_string(char *out, size_t out_len)
{
    pico_unique_board_id_t board_id;
    pico_get_unique_board_id(&board_id);
    static const char hex[] = "0123456789abcdef";

    if (out_len == 0) return;
    size_t pos = 0;
    const size_t prefix_len = strlen(FLEXRAY_SERIAL_PREFIX);
    if (prefix_len < out_len) {
        memcpy(out, FLEXRAY_SERIAL_PREFIX, prefix_len);
        pos = prefix_len;
    }
    for (int i = 0; i < 8 && (pos + 2) < out_len; i++) {
        out[pos++] = hex[board_id.id[i] >> 4];
        out[pos++] = hex[board_id.id[i] & 0x0F];
    }
    out[pos] = '\0';
}

static bool handle_panda_control_read(uint8_t rhport, tusb_control_request_t const *request)
{
    uint8_t response[128] = {0};
    uint16_t response_len = 0;

    switch (request->bRequest) {
    case PANDA_GET_HW_TYPE:
        response[0] = HW_TYPE_RED_PANDA;
        response_len = 1;
        break;

    case PANDA_GET_MICROSECOND_TIMER: {
        uint32_t now = time_us_32();
        memcpy(response, &now, sizeof(now));
        response_len = sizeof(now);
        break;
    }

    case PANDA_GET_CAN_HEALTH_STATS: {
        struct can_health_t health = {0};
        memcpy(response, &health, sizeof(health));
        response_len = sizeof(health);
        break;
    }

    case PANDA_GET_MCU_UID: {
        pico_unique_board_id_t board_id;
        pico_get_unique_board_id(&board_id);
        memcpy(response, board_id.id, PICO_UNIQUE_BOARD_ID_SIZE_BYTES);
        response_len = PICO_UNIQUE_BOARD_ID_SIZE_BYTES;
        break;
    }

    case PANDA_FETCH_SERIAL_NUMBER:
        make_serial_string((char *)response, sizeof(response));
        response_len = (uint16_t)strlen((char *)response);
        break;

    case PANDA_GET_HEALTH_PACKET: {
        struct health_t health = {0};
        health.uptime_pkt = to_ms_since_boot(get_absolute_time());
        health.ignition_line_pkt = 1;
        health.ignition_can_pkt = 1;
        health.controls_allowed_pkt = 1;
        health.car_harness_status_pkt = 1;
        health.safety_mode_pkt = safety_model;
        health.safety_param_pkt = safety_param;
        health.alternative_experience_pkt = alternative_experience;
        memcpy(response, &health, sizeof(health));
        response_len = sizeof(health);
        break;
    }

    case PANDA_GET_SIGNATURE_PART1:
    case PANDA_GET_SIGNATURE_PART2:
        response_len = 64;
        break;

    case PANDA_GET_GIT_VERSION:
        memcpy(response, "signal-gen", 10);
        response_len = 10;
        break;

    case PANDA_GET_VERSIONS:
        response[0] = 17;
        response[1] = 4;
        response[2] = 5;
        response_len = 3;
        break;

    case PANDA_UART_READ:
        response_len = 0;
        break;

    default:
        return false;
    }

    if (request->wLength < response_len) {
        response_len = request->wLength;
    }
    return tud_control_xfer(rhport, request, response, response_len);
}

static bool handle_panda_control_write(uint8_t rhport, tusb_control_request_t const *request)
{
    switch (request->bRequest) {
    case PANDA_RESET_CAN_COMMS:
    case PANDA_SET_OBD_CAN_MUX_MODE:
    case PANDA_SET_CAN_SPEED_KBPS:
    case PANDA_SET_CAN_FD_DATA_BITRATE:
    case PANDA_SET_CAN_LOOPBACK:
    case PANDA_SET_CLOCK_SOURCE_PARAMS:
    case PANDA_SET_POWER_SAVE_STATE:
    case PANDA_SET_CAN_FD_AUTO_SWITCH:
    case PANDA_CAN_CLEAR_BUFFER:
    case PANDA_HEARTBEAT:
    case PANDA_DISABLE_HEARTBEAT_CHECKS:
        return tud_control_status(rhport, request);

    case PANDA_SET_SAFETY_MODEL:
        safety_model = (uint8_t)request->wValue;
        safety_param = request->wIndex;
        return tud_control_status(rhport, request);

    case PANDA_SET_ALT_EXPERIENCE:
        alternative_experience = request->wValue;
        return tud_control_status(rhport, request);

    case PANDA_ENTER_BOOTLOADER_MODE:
        pending_bootloader = true;
        return tud_control_status(rhport, request);

    case PANDA_SYSTEM_RESET:
        pending_reset = true;
        return tud_control_status(rhport, request);

    default:
        return false;
    }
}

// ---------- Vendor control request handler ----------
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                 tusb_control_request_t const *request)
{
    if (stage == CONTROL_STAGE_ACK) {
        if (pending_reset) {
            watchdog_reboot(0, 0, 0);
        } else if (pending_bootloader) {
            reset_usb_boot(0, 0);
        }
        return true;
    }

    if (stage != CONTROL_STAGE_SETUP) return true;

    pending_reset = false;
    pending_bootloader = false;

    switch (request->bmRequestType_bit.type) {
    case TUSB_REQ_TYPE_VENDOR:
        if (request->bmRequestType & TUSB_DIR_IN_MASK) {
            return handle_panda_control_read(rhport, request);
        }
        if (request->wLength == 0) {
            return handle_panda_control_write(rhport, request);
        }
        break;

    case TUSB_REQ_TYPE_CLASS:
        if (request->bRequest == 0x22) {
            return tud_control_status(rhport, request);
        }
        break;

    default:
        break;
    }
    return false;
}
