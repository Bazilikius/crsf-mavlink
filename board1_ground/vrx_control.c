#include "ground_station.h"
#include <string.h>

#ifdef PICO_BOARD
#include "pico/stdlib.h"
#include "hardware/i2c.h"
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

// VRX I2C constant definitions
#define VRX_I2C_ADDR 0x35 // Common I2C target address for customizable video receivers

// Predefined RF bands & frequencies (MHz)
// 5 Bands x 8 Channels
static const uint16_t s_vrx_freq_table[5][8] = {
    {5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725}, // Band A
    {5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866}, // Band B
    {5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945}, // Band E
    {5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880}, // Band F (Fatshark)
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
#endif
    vrx_set_band_channel(g_config.vrx_band, g_config.vrx_channel);
}

// Convert frequency in MHz to RTC6715 synthesizer register values and send over I2C
void vrx_set_frequency(uint16_t mhz) {
    g_config.vrx_frequency_mhz = mhz;

    // SPI/I2C synth programming register formula for RTC6715 / RX5808:
    // f_LO = 479 + N * 2 + A * 2 / 32
    // R = 64 (Reference oscillator divider)
    // Register 0x0F contains synthesizer control data: PLL divider, etc.
    // For standard I2C receivers, we typically write the frequency or register bytes directly.
    uint32_t f_val = mhz - 479;
    uint32_t N = f_val / 2;
    uint32_t A = (f_val % 2) * 16;
    uint32_t reg_val = (A & 0x1F) | ((N & 0x1FF) << 5);

    uint8_t cmd[3];
    cmd[0] = 0x0F; // Address register
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
    vrx_set_frequency(mhz);
}

// Extract RC channels from packed CRSF channel payload
// CRSF channel payload contains 16 channels, 11-bits each, packed continuously.
void vrx_update_channels(const uint16_t channels[16]) {
    // Read the configured RC channel (1-indexed) to select VRX channel
    uint8_t chan_idx = g_config.vrx_rc_channel - 1;
    if (chan_idx >= 16) return;

    uint16_t val = channels[chan_idx]; // Typically ranges from 172 to 1811

    // Scale 172..1811 to 0..7 to select VRX channels
    if (val < 172) val = 172;
    if (val > 1811) val = 1811;

    uint32_t norm_val = val - 172; // 0..1639
    uint8_t selected_chan = (norm_val * 8) / 1640;
    if (selected_chan > 7) selected_chan = 7;

    if (selected_chan != g_config.vrx_channel) {
        vrx_set_band_channel(g_config.vrx_band, selected_chan);
        send_config_to_pc(); // Update GUI with new channel
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
            // CRSF length includes type byte + payload + checksum (which is 1 byte)
            // So payload size is length - 2
            if (s_crsf_payload_idx >= (s_crsf_len - 2)) {
                s_crsf_state = CRSF_STATE_CHECKSUM;
            }
            break;

        case CRSF_STATE_CHECKSUM:
            // Checksum parsed. Let's decode channels if type is CRSF_TYPE_CHANNELS
            if (s_crsf_type == CRSF_TYPE_CHANNELS && s_crsf_payload_idx >= 22) {
                // Decode 11-bit channels
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
