#include "rf_switcher.h"

#ifdef PICO_BOARD
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

// Basic PIO UART assembly code inline/loaded dynamically
// to avoid requiring external compiler-generated .pio.h header files.
// This is extremely robust and avoids build system mismatches!

// PIO UART TX instructions
static const uint16_t uart_tx_program_instructions[] = {
    0x9fa0, //  0: pull   block           side 1 [7]
    0xf701, //  1: set    pins, 1         side 0 [7]
    0x6001, //  2: out    pins, 1
    0x0642, //  3: jmp    !osre, 2        side 0 [6]
};

static const struct pio_program uart_tx_program = {
    .instructions = uart_tx_program_instructions,
    .length = 4,
    .origin = -1,
};

// PIO UART RX instructions
static const uint16_t uart_rx_program_instructions[] = {
    0x2020, //  0: wait   0 pin, 0
    0xea27, //  1: set    x, 7            side 0 [10]
    0x4001, //  2: in     pins, 1
    0x0082, //  3: jmp    x--, 2          side 0 [7]
};

static const struct pio_program uart_rx_program = {
    .instructions = uart_rx_program_instructions,
    .length = 4,
    .origin = -1,
};

static PIO s_pio = pio0;
static uint s_sm_tx = 0;
static uint s_sm_rx = 1;

void pio_uart_init(void) {
    // Add programs
    uint offset_tx = pio_add_program(s_pio, &uart_tx_program);
    uint offset_rx = pio_add_program(s_pio, &uart_rx_program);

    // Configure GPIO pins
    pio_gpio_init(s_pio, PIN_JR2_PIO_TX);
    pio_gpio_init(s_pio, PIN_JR2_PIO_RX);
    gpio_pull_up(PIN_JR2_PIO_RX);

    // Set directions
    pio_sm_set_consecutive_pindirs(s_pio, s_sm_tx, PIN_JR2_PIO_TX, 1, true);
    pio_sm_set_consecutive_pindirs(s_pio, s_sm_rx, PIN_JR2_PIO_RX, 1, false);

    // CRSF standard baud rate is 420000 bps
    float div = (float)clock_get_hz(clk_sys) / (8 * 420000);

    // Configure TX SM
    pio_sm_config c_tx = pio_get_default_sm_config();
    sm_config_set_out_pins(&c_tx, PIN_JR2_PIO_TX, 1);
    sm_config_set_sideset_pins(&c_tx, PIN_JR2_PIO_TX);
    sm_config_set_sideset(&c_tx, 1, false, false); // Configure 1 sideset bit
    sm_config_set_out_shift(&c_tx, true, false, 32);
    sm_config_set_clkdiv(&c_tx, div);
    pio_sm_init(s_pio, s_sm_tx, offset_tx, &c_tx);
    pio_sm_set_enabled(s_pio, s_sm_tx, true);

    // Configure RX SM
    pio_sm_config c_rx = pio_get_default_sm_config();
    sm_config_set_in_pins(&c_rx, PIN_JR2_PIO_RX);
    sm_config_set_in_shift(&c_rx, true, true, 8);
    sm_config_set_clkdiv(&c_rx, div);
    pio_sm_init(s_pio, s_sm_rx, offset_rx, &c_rx);
    pio_sm_set_enabled(s_pio, s_sm_rx, true);
}

void pio_uart_write(const uint8_t *buf, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        while (pio_sm_is_tx_fifo_full(s_pio, s_sm_tx));
        pio_sm_put(s_pio, s_sm_tx, buf[i]);
    }
}

uint16_t pio_uart_read(uint8_t *buf, uint16_t max_len) {
    uint16_t count = 0;
    while (!pio_sm_is_rx_fifo_empty(s_pio, s_sm_rx) && count < max_len) {
        buf[count++] = (uint8_t)(pio_sm_get(s_pio, s_sm_rx) >> 24);
    }
    return count;
}

#else
// Mock implementation for non-RP2040 host environments (such as testing)
static uint8_t mock_rx_buffer[256];
static uint16_t mock_rx_len = 0;

void pio_uart_init(void) {}
void pio_uart_write(const uint8_t *buf, uint16_t len) {
    (void)buf; (void)len;
}
uint16_t pio_uart_read(uint8_t *buf, uint16_t max_len) {
    uint16_t read_bytes = (mock_rx_len < max_len) ? mock_rx_len : max_len;
    if (read_bytes > 0) {
        for (uint16_t i = 0; i < read_bytes; i++) {
            buf[i] = mock_rx_buffer[i];
        }
        mock_rx_len -= read_bytes;
    }
    return read_bytes;
}
#endif
