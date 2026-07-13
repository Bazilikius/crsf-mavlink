#include "ground_station.h"
#include <string.h>

// Simulated or real includes based on build context
#ifdef PICO_BOARD
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "tusb.h"
#else
// For native/mock compiling and testing
#include <stdio.h>
#define uart_default ((void*)0)
#define uart_getc(...) (0)
#define uart_putc(...) ((void)0)
#define uart_is_writable(...) (true)
#define uart_is_readable(...) (false)
#define tud_cdc_n_write(...) (0)
#define tud_cdc_n_read(...) (0)
#define tud_cdc_n_write_flush(...) ((void)0)
#define tud_cdc_n_available(...) (0)
#endif

SystemConfig g_config;
MuxParser g_mux_parser;

void init_system(void) {
    g_config.system_mode = MODE_SIMULTANEOUS;

    g_config.azimuth_min_us = 1000;
    g_config.azimuth_max_us = 2000;
    g_config.azimuth_trim_us = 1500;
    g_config.azimuth_reversed = 0;

    g_config.elevation_min_us = 1000;
    g_config.elevation_max_us = 2000;
    g_config.elevation_trim_us = 1500;
    g_config.elevation_reversed = 0;

    g_config.home_lat = 0.0f;
    g_config.home_lon = 0.0f;
    g_config.home_alt = 0.0f;
    g_config.home_set = false;

    g_config.vrx_rc_channel = 8;
    g_config.vrx_band = 3;
    g_config.vrx_channel = 0;
    g_config.vrx_frequency_mhz = 5740;

    g_config.manual_override = 0;
    g_config.active_camera = 1;
    g_config.cam_rc_channel = 7;

    g_config.live_azimuth_deg = 0;
    g_config.live_elevation_deg = 0;

    mux_parser_init(&g_mux_parser);
}

// Format config and send back to PC over USB CDC 1
void send_config_to_pc(void) {
    uint8_t buffer[64];
    // Sync header: [0xCF, 0xFC]
    buffer[0] = 0xCF;
    buffer[1] = 0xFC;
    buffer[2] = g_config.system_mode;

    // Servo parameters
    buffer[3] = g_config.azimuth_min_us >> 8;
    buffer[4] = g_config.azimuth_min_us & 0xFF;
    buffer[5] = g_config.azimuth_max_us >> 8;
    buffer[6] = g_config.azimuth_max_us & 0xFF;
    buffer[7] = g_config.azimuth_trim_us >> 8;
    buffer[8] = g_config.azimuth_trim_us & 0xFF;
    buffer[9] = g_config.azimuth_reversed;

    buffer[10] = g_config.elevation_min_us >> 8;
    buffer[11] = g_config.elevation_min_us & 0xFF;
    buffer[12] = g_config.elevation_max_us >> 8;
    buffer[13] = g_config.elevation_max_us & 0xFF;
    buffer[14] = g_config.elevation_trim_us >> 8;
    buffer[15] = g_config.elevation_trim_us & 0xFF;
    buffer[16] = g_config.elevation_reversed;

    // Lat / Lon / Alt
    memcpy(&buffer[17], &g_config.home_lat, sizeof(float));
    memcpy(&buffer[21], &g_config.home_lon, sizeof(float));
    memcpy(&buffer[25], &g_config.home_alt, sizeof(float));
    buffer[29] = g_config.home_set ? 1 : 0;

    // VRX Parameters
    buffer[30] = g_config.vrx_rc_channel;
    buffer[31] = g_config.vrx_band;
    buffer[32] = g_config.vrx_channel;
    buffer[33] = g_config.vrx_frequency_mhz >> 8;
    buffer[34] = g_config.vrx_frequency_mhz & 0xFF;

    // New parameters
    buffer[35] = g_config.manual_override;
    buffer[36] = g_config.active_camera;
    buffer[37] = g_config.cam_rc_channel;
    buffer[38] = g_config.live_azimuth_deg >> 8;
    buffer[39] = g_config.live_azimuth_deg & 0xFF;
    buffer[40] = g_config.live_elevation_deg >> 8;
    buffer[41] = g_config.live_elevation_deg & 0xFF;

    // Calculate 8-bit checksum over payload bytes (index 2 to 41)
    uint8_t cksum = 0;
    for (uint8_t i = 2; i <= 41; i++) {
        cksum += buffer[i];
    }
    buffer[42] = cksum;

#ifdef PICO_BOARD
    tud_cdc_n_write(1, buffer, 43);
    tud_cdc_n_write_flush(1);
#endif
}

void process_pc_command(const uint8_t *payload, uint8_t len) {
    if (len < 1) return;

    uint8_t cmd_type = payload[0];
    switch (cmd_type) {
        case 0x10: // Set System Mode
            if (len >= 2) {
                g_config.system_mode = payload[1];
                // Forward mode change to Board 2 via multiplexer command
                uint8_t enc_buf[16];
                uint8_t cmd_payload[2] = {0x10, g_config.system_mode};
                uint16_t enc_len = mux_encode(MUX_CHAN_CONFIG, cmd_payload, 2, enc_buf);
#ifdef PICO_BOARD
                for (uint16_t i = 0; i < enc_len; i++) {
                    uart_putc(uart1, enc_buf[i]);
                }
#endif
            }
            break;

        case 0x20: // Calibration Data Set
            if (len >= 15) {
                g_config.azimuth_min_us = (payload[1] << 8) | payload[2];
                g_config.azimuth_max_us = (payload[3] << 8) | payload[4];
                g_config.azimuth_trim_us = (payload[5] << 8) | payload[6];
                g_config.azimuth_reversed = payload[7];

                g_config.elevation_min_us = (payload[8] << 8) | payload[9];
                g_config.elevation_max_us = (payload[10] << 8) | payload[11];
                g_config.elevation_trim_us = (payload[12] << 8) | payload[13];
                g_config.elevation_reversed = payload[14];
            }
            break;

        case 0x30: // Set Home Position
            if (len >= 13) {
                memcpy(&g_config.home_lat, &payload[1], sizeof(float));
                memcpy(&g_config.home_lon, &payload[5], sizeof(float));
                memcpy(&g_config.home_alt, &payload[9], sizeof(float));
                g_config.home_set = true;
            }
            break;

        case 0x40: // VRX Configuration Update
            if (len >= 5) {
                g_config.vrx_rc_channel = payload[1];
                g_config.vrx_band = payload[2];
                g_config.vrx_channel = payload[3];
                g_config.vrx_frequency_mhz = (payload[4] << 8) | payload[5];
                vrx_set_frequency(g_config.vrx_frequency_mhz);
            }
            break;

        case 0x50: // Read Config Request
            send_config_to_pc();
            break;

        case 0x60: // Set Camera Switch & Manual Potentiometer Settings
            if (len >= 4) {
                g_config.active_camera = payload[1];
                g_config.cam_rc_channel = payload[2];
                g_config.manual_override = payload[3];
                vrx_set_cam_switch(g_config.active_camera);
            }
            break;
    }
}
