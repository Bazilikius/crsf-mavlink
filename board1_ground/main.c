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
    oled_init();

    uint32_t last_display_update_ms = 0;

    while (1) {
#ifdef PICO_BOARD
        tud_task(); // Maintain TinyUSB device library tasks

        // ----------------- Potentiometer Readings & Display -----------------
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - last_display_update_ms >= 50) {
            last_display_update_ms = now_ms;

            // Read potentiometers (only updates PWM if manual override is active)
            tracker_read_potentiometers();

            // Update I2C SSD1306 display
            oled_update_display();
        }

        // ----------------- USB CDC 0 Routing (MAVLink) -----------------
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
        if (tud_cdc_n_available(1)) {
            uint8_t cmd_buf[64];
            uint32_t count = tud_cdc_n_read(1, cmd_buf, sizeof(cmd_buf));
            if (count > 0) {
                process_pc_command(cmd_buf, count);
            }
        }

        // ----------------- UART1 Routing (Data from Board 2) -----------------
        while (uart_is_readable(uart1)) {
            uint8_t b = uart_getc(uart1);
            MuxFrame rx_frame;
            if (mux_parse_byte(&g_mux_parser, b, &rx_frame)) {
                if (rx_frame.chan_id == MUX_CHAN_MAVLINK) {
                    tud_cdc_n_write(0, rx_frame.payload, rx_frame.len);
                    tud_cdc_n_write_flush(0);

                    for (uint8_t i = 0; i < rx_frame.len; i++) {
                        tracker_parse_mavlink_byte(rx_frame.payload[i]);
                    }
                }
                else if (rx_frame.chan_id == MUX_CHAN_CRSF) {
                    for (uint8_t i = 0; i < rx_frame.len; i++) {
                        vrx_parse_crsf_byte(rx_frame.payload[i]);
                    }
                }
                else if (rx_frame.chan_id == MUX_CHAN_CONFIG) {
                    process_pc_command(rx_frame.payload, rx_frame.len);
                }
            }
        }
#else
        break;
#endif
    }

    return 0;
}
