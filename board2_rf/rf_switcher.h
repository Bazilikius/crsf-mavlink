#ifndef RF_SWITCHER_H
#define RF_SWITCHER_H

#include <stdint.h>
#include <stdbool.h>
#include "../shared/mux_protocol.h"

// Hardware Pin Configuration for Board 2 (YD-RP2040)
#define PIN_UART1_TX      4    // High speed link to Board 1
#define PIN_UART1_RX      5
#define PIN_JR1_UART_TX   0    // UART0 to JR Module 1 (CRSF + MAVLink)
#define PIN_JR1_UART_RX   1
#define PIN_JR2_PIO_TX    8    // PIO UART to JR Module 2 (CRSF only)
#define PIN_JR2_PIO_RX    9

// Enable / Control lines for JR modules
#define PIN_JR1_PWR_EN    12
#define PIN_JR2_PWR_EN    13

// System Modes (synchronized with Board 1)
typedef enum {
    MODE_JR1_ALL = 1,       // JR Module 1 handles both CRSF and MAVLink
    MODE_JR2_CRSF = 2,      // JR Module 2 handles CRSF, JR Module 1 inactive
    MODE_SIMULTANEOUS = 3   // JR Module 1 handles MAVLink, JR Module 2 handles CRSF
} SwitcherMode;

extern uint8_t g_active_mode;
extern MuxParser g_mux_parser;

// Core APIs
void switcher_init(void);
void switcher_set_mode(uint8_t mode);
void switcher_process_command(const uint8_t *payload, uint8_t len);
void pio_uart_init(void);
void pio_uart_write(const uint8_t *buf, uint16_t len);
uint16_t pio_uart_read(uint8_t *buf, uint16_t max_len);

#endif // RF_SWITCHER_H
