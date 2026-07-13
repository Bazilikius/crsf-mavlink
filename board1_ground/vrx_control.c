#include "ground_station.h"
#include <string.h>

#ifdef PICO_BOARD
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#endif

#define CRSF_ADDR_FC          0xC8
#define CRSF_TYPE_CHANNELS    0x16

// Simple CRSF parsing state
typedef enum {
    CRSF_STATE_UNINIT = 0,
    CRSF_STATE_LEN,
    CRSF_STATE_TYPE,
    CRSF_STATE_PAYLOAD,
    CRSF_STATE_CHECKSUM
} CrsfParserState;

static CrsfParserState s_crsf_state = CRSF_STATE_UNINIT;
static uint8_t s_crsf_len = 0;
static uint8_t s_crsf_type = 0;
static uint8_t s_crsf_payload[64];
static uint8_t s_crsf_payload_idx = 0;

#define VRX_I2C_ADDR 0x35

static const uint16_t s_vrx_freq_table[5][8] = {
    {5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725}, // Band A
    {5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866}, // Band B
    {5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945}, // Band E
    {5740, 5760, 5780, 0x0000, 5820, 5840, 5860, 5880}, // Band F (Fatshark)
    {5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917}  // Raceband
};

void vrx_init(void) {
#ifdef PICO_BOARD
    // Initialize I2C0 at 100 kHz
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);

    // Initialize Cam Switch GPIO
    gpio_init(PIN_CAM_SWITCH);
    gpio_set_dir(PIN_CAM_SWITCH, GPIO_OUT);
#endif

    // Setup defaults
    g_config.manual_override = 0;
    g_config.active_camera = 1; // Default to VRX camera
    g_config.cam_rc_channel = 7; // Channel 7 default

    vrx_set_band_channel(g_config.vrx_band, g_config.vrx_channel);
    vrx_set_cam_switch(g_config.active_camera);
}

void vrx_set_frequency(uint16_t mhz) {
    g_config.vrx_frequency_mhz = mhz;

    uint32_t f_val = mhz - 479;
    uint32_t N = f_val / 2;
    uint32_t A = (f_val % 2) * 16;
    uint32_t reg_val = (A & 0x1F) | ((N & 0x1FF) << 5);

    uint8_t cmd[3];
    cmd[0] = 0x0F;
    cmd[1] = reg_val & 0xFF;
    cmd[2] = (reg_val >> 8) & 0xFF;

#ifdef PICO_BOARD
    i2c_write_blocking(i2c0, VRX_I2C_ADDR, cmd, 3, false);
#endif
}

void vrx_set_band_channel(uint8_t band, uint8_t channel) {
    if (band > 4) band = 4;
    if (channel > 7) channel = 7;

    g_config.vrx_band = band;
    g_config.vrx_channel = channel;
    uint16_t mhz = s_vrx_freq_table[band][channel];
    if (mhz > 0) {
        vrx_set_frequency(mhz);
    }
}

void vrx_set_cam_switch(uint8_t active_cam) {
    g_config.active_camera = active_cam;
#ifdef PICO_BOARD
    gpio_put(PIN_CAM_SWITCH, active_cam == 1 ? 1 : 0);
#endif
}

void vrx_update_channels(const uint16_t channels[16]) {
    // 1. Process VRX channel selection channel (1-indexed)
    uint8_t vrx_idx = g_config.vrx_rc_channel - 1;
    if (vrx_idx < 16) {
        uint16_t val = channels[vrx_idx];
        if (val >= 172 && val <= 1811) {
            uint32_t norm_val = val - 172;
            uint8_t selected_chan = (norm_val * 8) / 1640;
            if (selected_chan > 7) selected_chan = 7;

            if (selected_chan != g_config.vrx_channel) {
                vrx_set_band_channel(g_config.vrx_band, selected_chan);
                send_config_to_pc();
            }
        }
    }

    // 2. Process Cam Switch channel selection channel (1-indexed)
    uint8_t cam_idx = g_config.cam_rc_channel - 1;
    if (cam_idx < 16) {
        uint16_t val = channels[cam_idx];
        if (val >= 172 && val <= 1811) {
            // Neutral is 992. Toggle camera switch: High (> 992) = VRX, Low (< 992) = Analog Cam
            uint8_t target_cam = (val > 992) ? 1 : 0;
            if (target_cam != g_config.active_camera) {
                vrx_set_cam_switch(target_cam);
                send_config_to_pc();
            }
        }
    }
}

// Parses streaming CRSF bytes
void vrx_parse_crsf_byte(uint8_t byte) {
    switch (s_crsf_state) {
        case CRSF_STATE_UNINIT:
            if (byte == CRSF_ADDR_FC) {
                s_crsf_state = CRSF_STATE_LEN;
            }
            break;

        case CRSF_STATE_LEN:
            if (byte >= 2 && byte <= 62) {
                s_crsf_len = byte;
                s_crsf_state = CRSF_STATE_TYPE;
            } else {
                s_crsf_state = CRSF_STATE_UNINIT;
            }
            break;

        case CRSF_STATE_TYPE:
            s_crsf_type = byte;
            s_crsf_payload_idx = 0;
            if (s_crsf_len > 2) {
                s_crsf_state = CRSF_STATE_PAYLOAD;
            } else {
                s_crsf_state = CRSF_STATE_CHECKSUM;
            }
            break;

        case CRSF_STATE_PAYLOAD:
            s_crsf_payload[s_crsf_payload_idx++] = byte;
            if (s_crsf_payload_idx >= (s_crsf_len - 2)) {
                s_crsf_state = CRSF_STATE_CHECKSUM;
            }
            break;

        case CRSF_STATE_CHECKSUM:
            if (s_crsf_type == CRSF_TYPE_CHANNELS && s_crsf_payload_idx >= 22) {
                uint16_t channels[16];
                channels[0]  = (s_crsf_payload[0]       | s_crsf_payload[1]  << 8) & 0x07FF;
                channels[1]  = (s_crsf_payload[1]  >> 3 | s_crsf_payload[2]  << 5) & 0x07FF;
                channels[2]  = (s_crsf_payload[2]  >> 6 | s_crsf_payload[3]  << 2 | s_crsf_payload[4] << 10) & 0x07FF;
                channels[3]  = (s_crsf_payload[4]  >> 1 | s_crsf_payload[5]  << 7) & 0x07FF;
                channels[4]  = (s_crsf_payload[5]  >> 4 | s_crsf_payload[6]  << 4) & 0x07FF;
                channels[5]  = (s_crsf_payload[6]  >> 7 | s_crsf_payload[7]  << 1 | s_crsf_payload[8] << 9) & 0x07FF;
                channels[6]  = (s_crsf_payload[8]  >> 2 | s_crsf_payload[9]  << 6) & 0x07FF;
                channels[7]  = (s_crsf_payload[9]  >> 5 | s_crsf_payload[10] << 3) & 0x07FF;
                channels[8]  = (s_crsf_payload[11]      | s_crsf_payload[12] << 8) & 0x07FF;
                channels[9]  = (s_crsf_payload[12] >> 3 | s_crsf_payload[13] << 5) & 0x07FF;
                channels[10] = (s_crsf_payload[13] >> 6 | s_crsf_payload[14] << 2 | s_crsf_payload[15] << 10) & 0x07FF;
                channels[11] = (s_crsf_payload[15] >> 1 | s_crsf_payload[16] << 7) & 0x07FF;
                channels[12] = (s_crsf_payload[16] >> 4 | s_crsf_payload[17] << 4) & 0x07FF;
                channels[13] = (s_crsf_payload[17] >> 7 | s_crsf_payload[18] << 1 | s_crsf_payload[19] << 9) & 0x07FF;
                channels[14] = (s_crsf_payload[19] >> 2 | s_crsf_payload[20] << 6) & 0x07FF;
                channels[15] = (s_crsf_payload[20] >> 5 | s_crsf_payload[21] << 3) & 0x07FF;

                vrx_update_channels(channels);
            }
            s_crsf_state = CRSF_STATE_UNINIT;
            break;
    }
}
