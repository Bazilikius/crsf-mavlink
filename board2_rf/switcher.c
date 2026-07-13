#include "rf_switcher.h"
#include <string.h>

#ifdef PICO_BOARD
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#else
#define gpio_put(...) ((void)0)
#define uart0 ((void*)0)
#define uart_init(...) ((void)0)
#define gpio_set_function(...) ((void)0)
#define gpio_init(...) ((void)0)
#define gpio_set_dir(...) ((void)0)
#endif

uint8_t g_active_mode = MODE_SIMULTANEOUS; // Default mode matches Board 1
MuxParser g_mux_parser;

void switcher_init(void) {
#ifdef PICO_BOARD
    // Initialize standard outputs for power switches
    gpio_init(PIN_JR1_PWR_EN);
    gpio_set_dir(PIN_JR1_PWR_EN, GPIO_OUT);

    gpio_init(PIN_JR2_PWR_EN);
    gpio_set_dir(PIN_JR2_PWR_EN, GPIO_OUT);
#endif

    // Apply default mode state (Simultaneous operation)
    switcher_set_mode(g_active_mode);

    mux_parser_init(&g_mux_parser);
}

void switcher_set_mode(uint8_t mode) {
    g_active_mode = mode;

    switch (g_active_mode) {
        case MODE_JR1_ALL:
            // Power on JR1, Power off JR2
            gpio_put(PIN_JR1_PWR_EN, 1);
            gpio_put(PIN_JR2_PWR_EN, 0);
            break;

        case MODE_JR2_CRSF:
            // Power off JR1, Power on JR2
            gpio_put(PIN_JR1_PWR_EN, 0);
            gpio_put(PIN_JR2_PWR_EN, 1);
            break;

        case MODE_SIMULTANEOUS:
            // Power on both modules for simultaneous operation
            gpio_put(PIN_JR1_PWR_EN, 1);
            gpio_put(PIN_JR2_PWR_EN, 1);
            break;

        default:
            break;
    }
}

void switcher_process_command(const uint8_t *payload, uint8_t len) {
    if (len < 1) return;

    uint8_t cmd_type = payload[0];
    switch (cmd_type) {
        case 0x10: // Change mode command
            if (len >= 2) {
                switcher_set_mode(payload[1]);
            }
            break;
    }
}
