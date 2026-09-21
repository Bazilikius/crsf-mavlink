/**
 * Board 1 RP2040 C/C++ SDK Firmware
 * Bridges UART0 (GPIO 0 TX, GPIO 1 RX - MAVLink Device) and UART1 (GPIO 4 TX, GPIO 5 RX - Inter-board link)
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#define UART_DEVICE_ID uart0
#define UART_DEVICE_BAUD 115200
#define UART_DEVICE_TX_PIN 0
#define UART_DEVICE_RX_PIN 1

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

static ring_buffer_t dev_to_inter;
static ring_buffer_t inter_to_dev;

static inline bool ring_buf_put(ring_buffer_t *rb, uint8_t byte) {
    uint16_t next = (rb->head + 1) % RING_BUF_SIZE;
    if (next == rb->tail) return false; // Buffer full
    rb->buffer[rb->head] = byte;
    rb->head = next;
    return true;
}

static inline bool ring_buf_get(ring_buffer_t *rb, uint8_t *byte) {
    if (rb->head == rb->tail) return false; // Buffer empty
    *byte = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % RING_BUF_SIZE;
    return true;
}

void on_uart_device_rx() {
    while (uart_is_readable(UART_DEVICE_ID)) {
        uint8_t ch = uart_getc(UART_DEVICE_ID);
        ring_buf_put(&dev_to_inter, ch);
    }
}

void on_uart_interboard_rx() {
    while (uart_is_readable(UART_INTERBOARD_ID)) {
        uint8_t ch = uart_getc(UART_INTERBOARD_ID);
        ring_buf_put(&inter_to_dev, ch);
    }
}

int main() {
    // NOTE: stdio_init_all() intentionally NOT called. By default the Pico SDK maps
    // stdio to UART0 on GPIO 0/1, which are the pins wired to the MAVLink device.

    // Initialize UART0 for MAVLink Device
    uart_init(UART_DEVICE_ID, UART_DEVICE_BAUD);
    gpio_set_function(UART_DEVICE_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_DEVICE_RX_PIN, GPIO_FUNC_UART);

    // Initialize UART1 for Inter-board Link
    uart_init(UART_INTERBOARD_ID, UART_INTERBOARD_BAUD);
    gpio_set_function(UART_INTERBOARD_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_INTERBOARD_RX_PIN, GPIO_FUNC_UART);

    // Enable IRQ for UART reading
    int dev_irq = UART_DEVICE_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    int inter_irq = UART_INTERBOARD_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    irq_set_exclusive_handler(dev_irq, on_uart_device_rx);
    irq_set_enabled(dev_irq, true);
    uart_set_irq_enables(UART_DEVICE_ID, true, false);

    irq_set_exclusive_handler(inter_irq, on_uart_interboard_rx);
    irq_set_enabled(inter_irq, true);
    uart_set_irq_enables(UART_INTERBOARD_ID, true, false);

    uint8_t byte;
    while (1) {
        // Drain both directions every pass. Only pop a byte when the destination
        // TX FIFO can take it, so neither direction ever blocks the other.
        while (uart_is_writable(UART_INTERBOARD_ID) && ring_buf_get(&dev_to_inter, &byte)) {
            uart_putc_raw(UART_INTERBOARD_ID, byte);
        }
        while (uart_is_writable(UART_DEVICE_ID) && ring_buf_get(&inter_to_dev, &byte)) {
            uart_putc_raw(UART_DEVICE_ID, byte);
        }
    }

    return 0;
}
