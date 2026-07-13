#include "rf_switcher.h"
#include <string.h>

#ifdef PICO_BOARD
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#else
#define uart1 ((void*)0)
#define uart0 ((void*)0)
#define uart_is_writable(...) (true)
#define uart_putc(...) ((void)0)
#define uart_is_readable(...) (false)
#define uart_getc(...) (0)
#endif

// Local buffer for UART aggregation
static uint8_t s_local_rx_buf[256];

// Simple state machine to parse CRSF stream from JR1 in Mode 1
static uint8_t s_jr1_crsf_buf[64];
static uint8_t s_jr1_crsf_idx = 0;
static uint8_t s_jr1_crsf_state = 0; // 0: Idle, 1: Len, 2: Payload

// Simple state machine to parse MAVLink stream from JR1 in Mode 1
static uint8_t s_jr1_mav_buf[256];
static uint16_t s_jr1_mav_idx = 0;
static uint8_t s_jr1_mav_state = 0; // 0: Idle, 1: Len, 2: Header/Payload
static uint8_t s_jr1_mav_len = 0;
static bool s_jr1_mav_is_v2 = false;
static uint16_t s_jr1_mav_crc = 0xFFFF;
static uint16_t s_jr1_parsed_crc = 0;

// Standard MAVLink X.25 CRC calculation
static uint16_t mavlink_crc_accumulate(uint8_t data, uint16_t crc) {
    uint8_t tmp = data ^ (uint8_t)(crc & 0xFF);
    tmp ^= (tmp << 4);
    return (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

// Standard CRSF CRC-8 calculation
static uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= ptr[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// Packet packing helper
static void send_mux_frame(uint8_t chan_id, const uint8_t *payload, uint8_t len) {
    uint8_t enc_buf[300];
    uint16_t enc_len = mux_encode(chan_id, payload, len, enc_buf);
#ifdef PICO_BOARD
    for (uint16_t i = 0; i < enc_len; i++) {
        uart_putc(uart1, enc_buf[i]);
    }
#endif
}

// Separate MAVLink and CRSF data from JR Module 1 when run in Mode 1
static void parse_and_forward_jr1_mixed(uint8_t byte) {
    // ---- CRSF Parsing with strict CRC validation ----
    switch (s_jr1_crsf_state) {
        case 0: // Sync/Addr
            if (byte == 0xC8 || byte == 0xEE || byte == 0xEA) {
                s_jr1_crsf_buf[0] = byte;
                s_jr1_crsf_idx = 1;
                s_jr1_crsf_state = 1;
            }
            break;
        case 1: // Length
            if (byte >= 2 && byte <= 62) {
                s_jr1_crsf_buf[1] = byte;
                s_jr1_crsf_idx = 2;
                s_jr1_crsf_state = 2;
            } else {
                s_jr1_crsf_state = 0;
            }
            break;
        case 2: // Payload + Checksum
            s_jr1_crsf_buf[s_jr1_crsf_idx++] = byte;
            // Total frame size = length + 2 (addr + len)
            if (s_jr1_crsf_idx >= (s_jr1_crsf_buf[1] + 2)) {
                // Calculate and validate CRSF CRC-8 on the payload bytes (from type at index 2 to payload)
                uint8_t payload_len = s_jr1_crsf_buf[1] - 1; // len includes type byte
                uint8_t calculated_crc = crsf_crc8(&s_jr1_crsf_buf[2], payload_len);
                uint8_t parsed_crc = s_jr1_crsf_buf[s_jr1_crsf_idx - 1];

                if (calculated_crc == parsed_crc) {
                    send_mux_frame(MUX_CHAN_CRSF, s_jr1_crsf_buf, s_jr1_crsf_idx);
                }
                s_jr1_crsf_state = 0;
            }
            break;
    }

    // ---- MAVLink Parsing with strict X.25 CRC validation ----
    switch (s_jr1_mav_state) {
        case 0: // Sync
            if (byte == 0xFE) { // MAVLink v1
                s_jr1_mav_is_v2 = false;
                s_jr1_mav_crc = 0xFFFF;
                s_jr1_mav_buf[0] = byte;
                s_jr1_mav_idx = 1;
                s_jr1_mav_state = 1;
            } else if (byte == 0xFD) { // MAVLink v2
                s_jr1_mav_is_v2 = true;
                s_jr1_mav_crc = 0xFFFF;
                s_jr1_mav_buf[0] = byte;
                s_jr1_mav_idx = 1;
                s_jr1_mav_state = 1;
            }
            break;
        case 1: // Length
            s_jr1_mav_len = byte;
            s_jr1_mav_crc = mavlink_crc_accumulate(byte, s_jr1_mav_crc);
            s_jr1_mav_buf[s_jr1_mav_idx++] = byte;
            s_jr1_mav_state = 2;
            break;
        case 2: // Rest of Header + Payload + Checksum (2 bytes)
            s_jr1_mav_buf[s_jr1_mav_idx++] = byte;

            // Accumulate CRC on header + payload (except start byte, length, and CRC bytes themselves)
            // Header for v1 without STX and LEN has 4 bytes (SEQ, SYSID, COMPID, MSGID)
            // Header for v2 without STX and LEN has 7 bytes (INC_FLAGS, COMP_FLAGS, SEQ, SYSID, COMPID, MSGID0, MSGID1, MSGID2)
            uint16_t total_header_len = s_jr1_mav_is_v2 ? 9 : 5;
            uint16_t payload_and_crc_start_idx = total_header_len + 1; // index after header (which is STX + LEN + other header fields)

            if (s_jr1_mav_idx < payload_and_crc_start_idx + s_jr1_mav_len) {
                s_jr1_mav_crc = mavlink_crc_accumulate(byte, s_jr1_mav_crc);
            }

            uint16_t target_len = s_jr1_mav_is_v2 ? (s_jr1_mav_len + 12) : (s_jr1_mav_len + 8);
            if (s_jr1_mav_idx >= target_len) {
                // Read checksum
                s_jr1_parsed_crc = s_jr1_mav_buf[target_len - 2] | (s_jr1_mav_buf[target_len - 1] << 8);

                // Get MSG_ID to find the extra CRC byte
                uint32_t msg_id = 0;
                if (s_jr1_mav_is_v2) {
                    msg_id = s_jr1_mav_buf[7] | (s_jr1_mav_buf[8] << 8) | (s_jr1_mav_buf[9] << 16);
                } else {
                    msg_id = s_jr1_mav_buf[5];
                }

                uint8_t extra_byte = 0;
                if (msg_id == 33) extra_byte = 104; // GLOBAL_POSITION_INT CRC Extra

                uint16_t final_crc = mavlink_crc_accumulate(extra_byte, s_jr1_mav_crc);
                if (final_crc == s_jr1_parsed_crc) {
                    send_mux_frame(MUX_CHAN_MAVLINK, s_jr1_mav_buf, s_jr1_mav_idx);
                }
                s_jr1_mav_state = 0;
            }
            break;
    }
}

int main(void) {
#ifdef PICO_BOARD
    stdio_init_all();

    // High-speed link to Board 1 (UART1)
    uart_init(uart1, 460800);
    gpio_set_function(PIN_UART1_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART1_RX, GPIO_FUNC_UART);

    // Hardware UART to JR Module 1 (UART0)
    uart_init(uart0, 115200); // MAVLink/CRSF speed setup
    gpio_set_function(PIN_JR1_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_JR1_UART_RX, GPIO_FUNC_UART);
#endif

    pio_uart_init();
    switcher_init();

    while (1) {
#ifdef PICO_BOARD
        // ----------------- Step 1: Handle Board 1 Inputs (UART1) -----------------
        while (uart_is_readable(uart1)) {
            uint8_t b = uart_getc(uart1);
            MuxFrame rx_frame;
            if (mux_parse_byte(&g_mux_parser, b, &rx_frame)) {
                if (rx_frame.chan_id == MUX_CHAN_MAVLINK) {
                    // Send to JR1 if mode is 1 or 3
                    if (g_active_mode == MODE_JR1_ALL || g_active_mode == MODE_SIMULTANEOUS) {
                        for (uint8_t i = 0; i < rx_frame.len; i++) {
                            uart_putc(uart0, rx_frame.payload[i]);
                        }
                    }
                }
                else if (rx_frame.chan_id == MUX_CHAN_CRSF) {
                    // Send to active CRSF module
                    if (g_active_mode == MODE_JR1_ALL) {
                        for (uint8_t i = 0; i < rx_frame.len; i++) {
                            uart_putc(uart0, rx_frame.payload[i]);
                        }
                    } else if (g_active_mode == MODE_JR2_CRSF || g_active_mode == MODE_SIMULTANEOUS) {
                        pio_uart_write(rx_frame.payload, rx_frame.len);
                    }
                }
                else if (rx_frame.chan_id == MUX_CHAN_CONFIG) {
                    // Config command
                    switcher_process_command(rx_frame.payload, rx_frame.len);
                }
            }
        }

        // ----------------- Step 2: Handle JR Module Inputs & Forward to Board 1 -----------------
        if (g_active_mode == MODE_JR1_ALL) {
            // JR Module 1 carries mixed MAVLink and CRSF data. Use parsing filters.
            while (uart_is_readable(uart0)) {
                uint8_t b = uart_getc(uart0);
                parse_and_forward_jr1_mixed(b);
            }
        }
        else if (g_active_mode == MODE_JR2_CRSF) {
            // JR Module 2 handles CRSF exclusively
            uint16_t count = pio_uart_read(s_local_rx_buf, sizeof(s_local_rx_buf));
            if (count > 0) {
                send_mux_frame(MUX_CHAN_CRSF, s_local_rx_buf, count);
            }
        }
        else if (g_active_mode == MODE_SIMULTANEOUS) {
            // JR Module 1 handles MAVLink only
            uint8_t chunk[64];
            uint8_t chunk_idx = 0;
            while (uart_is_readable(uart0) && chunk_idx < sizeof(chunk)) {
                chunk[chunk_idx++] = uart_getc(uart0);
            }
            if (chunk_idx > 0) {
                send_mux_frame(MUX_CHAN_MAVLINK, chunk, chunk_idx);
            }

            // JR Module 2 handles CRSF only
            uint16_t count = pio_uart_read(s_local_rx_buf, sizeof(s_local_rx_buf));
            if (count > 0) {
                send_mux_frame(MUX_CHAN_CRSF, s_local_rx_buf, count);
            }
        }
#else
        break;
#endif
    }

    return 0;
}
