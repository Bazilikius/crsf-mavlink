/**
 * Board 2 RP2040 C/C++ SDK Firmware
 * Bridges UART1 (GPIO 4 TX, GPIO 5 RX - Inter-board link) and USB CDC (PC connection)
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#define UART_INTERBOARD_ID uart1
#define UART_INTERBOARD_BAUD 115200
#define UART_INTERBOARD_TX_PIN 4
#define UART_INTERBOARD_RX_PIN 5

#define RING_BUF_SIZE 2048

typedef struct {
    uint8_t buffer[RING_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} ring_buffer_t;

static ring_buffer_t inter_to_usb;

static inline bool ring_buf_put(ring_buffer_t *rb, uint8_t byte) {
    uint16_t next = (rb->head + 1) % RING_BUF_SIZE;
    if (next == rb->tail) return false;
    rb->buffer[rb->head] = byte;
    rb->head = next;
    return true;
}

static inline bool ring_buf_get(ring_buffer_t *rb, uint8_t *byte) {
    if (rb->head == rb->tail) return false;
    *byte = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % RING_BUF_SIZE;
    return true;
}

void on_uart_interboard_rx() {
    while (uart_is_readable(UART_INTERBOARD_ID)) {
        uint8_t ch = uart_getc(UART_INTERBOARD_ID);
        ring_buf_put(&inter_to_usb, ch);
    }
}

int main() {
    stdio_init_all();

    // Disable CRLF translation for raw binary MAVLink communication over USB CDC
    stdio_set_translate_crlf(&stdio_usb, false);

    // Initialize UART1 for Inter-board Link
    uart_init(UART_INTERBOARD_ID, UART_INTERBOARD_BAUD);
    gpio_set_function(UART_INTERBOARD_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_INTERBOARD_RX_PIN, GPIO_FUNC_UART);

    int inter_irq = UART_INTERBOARD_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(inter_irq, on_uart_interboard_rx);
    irq_set_enabled(inter_irq, true);
    uart_set_irq_enables(UART_INTERBOARD_ID, true, false);

    uint8_t byte;
    while (1) {
        // Forward Inter-board UART -> USB CDC (PC)
        if (ring_buf_get(&inter_to_usb, &byte)) {
            putchar_raw(byte);
        }

        // Forward USB CDC (PC) -> Inter-board UART
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            while (!uart_is_writable(UART_INTERBOARD_ID)) tight_loop_contents();
            uart_putc_raw(UART_INTERBOARD_ID, (uint8_t)c);
        }
    }

    return 0;
}
