#include "ground_station.h"
#include <string.h>

#ifdef PICO_BOARD
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "tusb.h"
#endif

#ifndef PICO_BOARD
#define uart1 ((void*)0)
#endif

// Main entry point for Board 1 (Ground Station)
int main(void) {
#ifdef PICO_BOARD
    // Initialize Pico SDK Standard I/O
    stdio_init_all();
    tusb_init();

    // Initialize UART1 at high baudrate for duplex communication with Board 2
    uart_init(uart1, 460800);
    gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART_RX, GPIO_FUNC_UART);
#endif

    init_system();
    tracker_init();
    vrx_init();

    while (1) {
#ifdef PICO_BOARD
        tud_task(); // Maintain TinyUSB device library tasks

        // ----------------- USB CDC 0 Routing (MAVLink) -----------------
        // 1. Data from PC MAVLink USB Port (CDC 0) -> Send to Multiplexer (UART1)
        if (tud_cdc_n_available(0)) {
            uint8_t usb_buf[64];
            uint32_t count = tud_cdc_n_read(0, usb_buf, sizeof(usb_buf));
            if (count > 0) {
                uint8_t enc_buf[128];
                uint16_t enc_len = mux_encode(MUX_CHAN_MAVLINK, usb_buf, count, enc_buf);
                for (uint16_t i = 0; i < enc_len; i++) {
                    uart_putc(uart1, enc_buf[i]);
                }
            }
        }

        // ----------------- USB CDC 1 Routing (Configuration/Control) -----------------
        // 2. Data from PC Control/GUI Port (CDC 1) -> Process command locally
        if (tud_cdc_n_available(1)) {
            uint8_t cmd_buf[64];
            uint32_t count = tud_cdc_n_read(1, cmd_buf, sizeof(cmd_buf));
            if (count > 0) {
                process_pc_command(cmd_buf, count);
            }
        }

        // ----------------- UART1 Routing (Data from Board 2) -----------------
        // 3. Demultiplex data received from Board 2
        while (uart_is_readable(uart1)) {
            uint8_t b = uart_getc(uart1);
            MuxFrame rx_frame;
            if (mux_parse_byte(&g_mux_parser, b, &rx_frame)) {
                // Handle different virtual channels
                if (rx_frame.chan_id == MUX_CHAN_MAVLINK) {
                    // Send to CDC 0 (MAVLink COM port on Windows)
                    tud_cdc_n_write(0, rx_frame.payload, rx_frame.len);
                    tud_cdc_n_write_flush(0);

                    // Parse locally for Antenna Tracker
                    for (uint8_t i = 0; i < rx_frame.len; i++) {
                        tracker_parse_mavlink_byte(rx_frame.payload[i]);
                    }
                }
                else if (rx_frame.chan_id == MUX_CHAN_CRSF) {
                    // Parse CRSF channels to change VRX channels
                    for (uint8_t i = 0; i < rx_frame.len; i++) {
                        vrx_parse_crsf_byte(rx_frame.payload[i]);
                    }
                }
                else if (rx_frame.chan_id == MUX_CHAN_CONFIG) {
                    // Process external configuration or stats
                    process_pc_command(rx_frame.payload, rx_frame.len);
                }
            }
        }
#else
        // Sleep or break in non-microcontroller environment to avoid CPU spinning
        break;
#endif
    }

    return 0;
}
