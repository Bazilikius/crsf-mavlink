#ifndef GROUND_STATION_H
#define GROUND_STATION_H

#include <stdint.h>
#include <stdbool.h>
#include "../shared/mux_protocol.h"

// Hardware Configuration Pins for Board 1 (YD-RP2040)
#define PIN_SERVO_AZIMUTH   14
#define PIN_SERVO_ELEVATION 15
#define PIN_I2C_SDA         16
#define PIN_I2C_SCL         17
#define PIN_UART_TX         4
#define PIN_UART_RX         5

// Active System Mode
typedef enum {
    MODE_JR1_ALL = 1,       // JR Module 1 active (CRSF + MAVLink)
    MODE_JR2_CRSF = 2,      // JR Module 2 active (CRSF only)
    MODE_SIMULTANEOUS = 3   // JR Module 1 (MAVLink only), JR Module 2 (CRSF only)
} SystemMode;

// Config / Calibration parameters stored on board
typedef struct {
    uint8_t system_mode;

    // Servo limits
    uint16_t azimuth_min_us;
    uint16_t azimuth_max_us;
    uint16_t azimuth_trim_us;
    uint8_t azimuth_reversed;

    uint16_t elevation_min_us;
    uint16_t elevation_max_us;
    uint16_t elevation_trim_us;
    uint8_t elevation_reversed;

    // Tracking parameters
    float home_lat;
    float home_lon;
    float home_alt;
    bool home_set;

    // VRX Parameters
    uint8_t vrx_rc_channel;      // 1-indexed RC channel to select VRX band/freq (e.g. channel 8)
    uint8_t vrx_band;            // 0: A, 1: B, 2: E, 3: F, 4: Raceband
    uint8_t vrx_channel;         // 0 to 7
    uint16_t vrx_frequency_mhz;  // Frequency in MHz
} SystemConfig;

// Global structures
extern SystemConfig g_config;
extern MuxParser g_mux_parser;

// Core functions
void init_system(void);
void process_pc_command(const uint8_t *payload, uint8_t len);
void send_config_to_pc(void);

// Routing module
void route_usb_to_mux(void);
void route_mux_to_usb(void);

// Tracker module
void tracker_init(void);
void tracker_update_pwm(uint16_t azimuth_us, uint16_t elevation_us);
void tracker_parse_mavlink_byte(uint8_t byte);
void tracker_set_home(float lat, float lon, float alt);
void tracker_calculate_angles(float target_lat, float target_lon, float target_alt, uint16_t *out_az_us, uint16_t *out_el_us);

// VRX Control module
void vrx_init(void);
void vrx_set_frequency(uint16_t mhz);
void vrx_set_band_channel(uint8_t band, uint8_t channel);
void vrx_parse_crsf_byte(uint8_t byte);
void vrx_update_channels(const uint16_t channels[16]);

#endif // GROUND_STATION_H
