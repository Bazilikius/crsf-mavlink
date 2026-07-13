#include "oled.h"
#include "ground_station.h"
#include <string.h>
#include <stdio.h>

#ifdef PICO_BOARD
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#endif

#define OLED_ADDR 0x3C

// Standard 5x7 Font Table
static const uint8_t font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space
    {0x00, 0x00, 0x5f, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7f, 0x14, 0x7f, 0x14}, // #
    {0x24, 0x2a, 0x7f, 0x2a, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1c, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1c, 0x00}, // )
    {0x14, 0x08, 0x3e, 0x08, 0x14}, // *
    {0x08, 0x08, 0x3e, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3e, 0x51, 0x49, 0x45, 0x3e}, // 0
    {0x00, 0x42, 0x7f, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4b, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7f, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3c, 0x4a, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1e}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3e}, // @
    {0x7e, 0x11, 0x11, 0x11, 0x7e}, // A
    {0x7f, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3e, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7f, 0x41, 0x41, 0x22, 0x1c}, // D
    {0x7f, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7f, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3e, 0x41, 0x49, 0x49, 0x7a}, // G
    {0x7f, 0x08, 0x08, 0x08, 0x7f}, // H
    {0x00, 0x41, 0x7f, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3f, 0x01}, // J
    {0x7f, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7f, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7f, 0x02, 0x0c, 0x02, 0x7f}, // M
    {0x7f, 0x04, 0x08, 0x10, 0x7f}, // N
    {0x3e, 0x41, 0x41, 0x41, 0x3e}, // O
    {0x7f, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3e, 0x41, 0x51, 0x21, 0x5e}, // Q
    {0x7f, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7f, 0x01, 0x01}, // T
    {0x3f, 0x40, 0x40, 0x40, 0x3f}, // U
    {0x1f, 0x20, 0x40, 0x20, 0x1f}, // V
    {0x3f, 0x40, 0x38, 0x40, 0x3f}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}  // Z
};

static void oled_send_cmd(uint8_t cmd) {
#ifdef PICO_BOARD
    uint8_t buf[2] = {0x00, cmd};
    i2c_write_blocking(i2c0, OLED_ADDR, buf, 2, false);
#endif
}

static void oled_send_data(const uint8_t *data, size_t len) {
#ifdef PICO_BOARD
    uint8_t buf[257];
    buf[0] = 0x40; // Co=0, D/C#=1 (Data)
    memcpy(&buf[1], data, len);
    i2c_write_blocking(i2c0, OLED_ADDR, buf, len + 1, false);
#endif
}

void oled_init(void) {
#ifdef PICO_BOARD
    // SSD1306 OLED Initialization sequence
    oled_send_cmd(0xAE); // Turn display off
    oled_send_cmd(0xD5); // Set display clock divide ratio/oscillator frequency
    oled_send_cmd(0x80);
    oled_send_cmd(0xA8); // Set multiplex ratio
    oled_send_cmd(0x3F); // 1/64 duty
    oled_send_cmd(0xD3); // Set display offset
    oled_send_cmd(0x00);
    oled_send_cmd(0x40); // Set display start line
    oled_send_cmd(0x8D); // Charge pump
    oled_send_cmd(0x14); // Enable charge pump
    oled_send_cmd(0x20); // Set memory addressing mode
    oled_send_cmd(0x02); // Page addressing mode
    oled_send_cmd(0xA1); // Segment remap (COL127 to SEG0)
    oled_send_cmd(0xC8); // COM scan direction (COM63 to COM0)
    oled_send_cmd(0xDA); // Set COM pins hardware configuration
    oled_send_cmd(0x12);
    oled_send_cmd(0x81); // Contrast control
    oled_send_cmd(0xCF);
    oled_send_cmd(0xD9); // Set pre-charge period
    oled_send_cmd(0xF1);
    oled_send_cmd(0xDB); // Set VCOMH deselect level
    oled_send_cmd(0x40);
    oled_send_cmd(0xA4); // Entire display on
    oled_send_cmd(0xA6); // Normal display mode (not inverted)
    oled_send_cmd(0xAF); // Turn display on
#endif
    oled_clear();
}

void oled_clear(void) {
    uint8_t zero_buf[128];
    memset(zero_buf, 0, sizeof(zero_buf));
    for (uint8_t page = 0; page < 8; page++) {
        oled_send_cmd(0xB0 + page); // Set Page address
        oled_send_cmd(0x00);        // Set Lower Column Start Address
        oled_send_cmd(0x10);        // Set Higher Column Start Address
        oled_send_data(zero_buf, 128);
    }
}

void oled_write_string(uint8_t col, uint8_t page, const char *str) {
    if (page > 7) return;
    oled_send_cmd(0xB0 + page);
    oled_send_cmd(col & 0x0F);
    oled_send_cmd(0x10 | ((col >> 4) & 0x0F));

    size_t len = strlen(str);
    for (size_t i = 0; i < len; i++) {
        char c = str[i];
        const uint8_t *char_pattern;

        if (c >= ' ' && c <= 'Z') {
            char_pattern = font5x7[c - ' '];
        } else if (c >= 'a' && c <= 'z') {
            // Map lower case to upper case pattern for 5x7 font simplicity
            char_pattern = font5x7[c - 'a' + 'A' - ' '];
        } else {
            char_pattern = font5x7[0]; // space
        }

        uint8_t pattern_buf[6];
        memcpy(pattern_buf, char_pattern, 5);
        pattern_buf[5] = 0x00; // character spacing gap
        oled_send_data(pattern_buf, 6);
    }
}

void oled_update_display(void) {
    char buf[22];

    // Line 0: Header
    oled_write_string(0, 0, "=== GS TRACKER ===");

    // Line 1: Operational Mode
    const char *mode_str = "Simultaneous";
    if (g_config.system_mode == MODE_JR1_ALL) mode_str = "JR1 Only";
    else if (g_config.system_mode == MODE_JR2_CRSF) mode_str = "JR2 Only";
    snprintf(buf, sizeof(buf), "MODE: %-13s", mode_str);
    oled_write_string(0, 1, buf);

    // Line 2: Azimuth Position
    const char *over_str = (g_config.manual_override == 1) ? "Pot" : "Auto";
    snprintf(buf, sizeof(buf), "AZ: %-3d deg (%s)", g_config.live_azimuth_deg, over_str);
    oled_write_string(0, 2, buf);

    // Line 3: Elevation Position
    snprintf(buf, sizeof(buf), "EL: %-3d deg", g_config.live_elevation_deg);
    oled_write_string(0, 3, buf);

    // Line 4: Camera Switch state
    const char *cam_str = (g_config.active_camera == 1) ? "VRX" : "Analog";
    snprintf(buf, sizeof(buf), "CAM: %-13s", cam_str);
    oled_write_string(0, 4, buf);

    // Line 5: Frequency info
    uint8_t band_char = 'A' + g_config.vrx_band;
    if (g_config.vrx_band == 3) band_char = 'F';
    else if (g_config.vrx_band == 4) band_char = 'R';
    snprintf(buf, sizeof(buf), "FRQ: %d (B%c C%d)", g_config.vrx_frequency_mhz, band_char, g_config.vrx_channel + 1);
    oled_write_string(0, 5, buf);
}
