#include "ground_station.h"
#include <math.h>
#include <string.h>

#ifdef PICO_BOARD
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Simple MAVLink v1/v2 parser state machine
typedef enum {
    MAV_STATE_UNINIT = 0,
    MAV_STATE_GOT_STX,
    MAV_STATE_GOT_LEN,
    MAV_STATE_GOT_INCOMPAT,
    MAV_STATE_GOT_COMPAT,
    MAV_STATE_GOT_SEQ,
    MAV_STATE_GOT_SYSID,
    MAV_STATE_GOT_COMPID,
    MAV_STATE_GOT_MSGID0,
    MAV_STATE_GOT_MSGID1,
    MAV_STATE_GOT_MSGID2,
    MAV_STATE_PAYLOAD,
    MAV_STATE_CHECKSUM_LSB,
    MAV_STATE_CHECKSUM_MSB
} MavParserState;

static MavParserState s_mav_state = MAV_STATE_UNINIT;
static uint8_t s_mav_len = 0;
static uint32_t s_mav_msg_id = 0;
static uint8_t s_mav_payload[256];
static uint16_t s_mav_payload_idx = 0;
static uint8_t s_mav_seq = 0;
static uint8_t s_is_v2 = false;
static uint16_t s_mav_crc = 0xFFFF;
static uint16_t s_parsed_crc = 0;

// Standard MAVLink X.25 CRC calculation
static uint16_t mavlink_crc_accumulate(uint8_t data, uint16_t crc) {
    uint8_t tmp = data ^ (uint8_t)(crc & 0xFF);
    tmp ^= (tmp << 4);
    return (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

void tracker_init(void) {
#ifdef PICO_BOARD
    // Initialize PWM pins for Azimuth and Elevation servos
    gpio_set_function(PIN_SERVO_AZIMUTH, GPIO_FUNC_PWM);
    gpio_set_function(PIN_SERVO_ELEVATION, GPIO_FUNC_PWM);

    // Set PWM wrap and clock div to produce 50Hz (20ms period)
    uint slice_az = pwm_gpio_to_slice_num(PIN_SERVO_AZIMUTH);
    uint slice_el = pwm_gpio_to_slice_num(PIN_SERVO_ELEVATION);

    // Assuming 125MHz system clock
    // 50Hz frequency -> 125MHz / (50 * 20000) = 125 system clock division
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 125.0f);
    pwm_config_set_wrap(&config, 20000); // 20000 cycles of 1us = 20ms (50Hz)

    pwm_init(slice_az, &config, true);
    pwm_init(slice_el, &config, true);

    // Set to mid position (1500us) initially
    tracker_update_pwm(g_config.azimuth_trim_us, g_config.elevation_trim_us);
#endif
}

void tracker_update_pwm(uint16_t azimuth_us, uint16_t elevation_us) {
    // Clamp to config safe limits
    if (azimuth_us < g_config.azimuth_min_us) azimuth_us = g_config.azimuth_min_us;
    if (azimuth_us > g_config.azimuth_max_us) azimuth_us = g_config.azimuth_max_us;

    if (elevation_us < g_config.elevation_min_us) elevation_us = g_config.elevation_min_us;
    if (elevation_us > g_config.elevation_max_us) elevation_us = g_config.elevation_max_us;

#ifdef PICO_BOARD
    pwm_set_gpio_level(PIN_SERVO_AZIMUTH, azimuth_us);
    pwm_set_gpio_level(PIN_SERVO_ELEVATION, elevation_us);
#endif
}

void tracker_set_home(float lat, float lon, float alt) {
    g_config.home_lat = lat;
    g_config.home_lon = lon;
    g_config.home_alt = alt;
    g_config.home_set = true;
}

void tracker_calculate_angles(float target_lat, float target_lon, float target_alt, uint16_t *out_az_us, uint16_t *out_el_us) {
    if (!g_config.home_set) {
        // If home is not set, default to neutral
        *out_az_us = g_config.azimuth_trim_us;
        *out_el_us = g_config.elevation_trim_us;
        return;
    }

    // Flat-Earth Approximation for short/medium ranges
    float lat_rad = g_config.home_lat * (M_PI / 180.0f);
    float d_lat = target_lat - g_config.home_lat;
    float d_lon = target_lon - g_config.home_lon;

    float y = d_lat * 111139.0f;
    float x = d_lon * 111139.0f * cosf(lat_rad);
    float z = target_alt - g_config.home_alt;

    // Calculate Azimuth (0 to 360 degrees)
    float azimuth_deg = atan2f(x, y) * (180.0f / M_PI);
    if (azimuth_deg < 0) {
        azimuth_deg += 360.0f;
    }

    // Calculate Distance and Elevation
    float horizontal_dist = sqrtf(x*x + y*y);
    float elevation_deg = 0.0f;
    if (horizontal_dist > 0.1f) {
        elevation_deg = atan2f(z, horizontal_dist) * (180.0f / M_PI);
    }
    if (elevation_deg < 0.0f) elevation_deg = 0.0f;
    if (elevation_deg > 90.0f) elevation_deg = 90.0f;

    // Map bearing (0 to 360 deg) to servo microseconds
    float az_range_us = g_config.azimuth_max_us - g_config.azimuth_min_us;
    float az_pct = azimuth_deg / 360.0f;
    if (g_config.azimuth_reversed) {
        az_pct = 1.0f - az_pct;
    }
    *out_az_us = g_config.azimuth_min_us + (uint16_t)(az_pct * az_range_us);

    // Map elevation (0 to 90 deg) to servo microseconds
    float el_range_us = g_config.elevation_max_us - g_config.elevation_min_us;
    float el_pct = elevation_deg / 90.0f;
    if (g_config.elevation_reversed) {
        el_pct = 1.0f - el_pct;
    }
    *out_el_us = g_config.elevation_min_us + (uint16_t)(el_pct * el_range_us);
}

// Simple internal handler when a valid MAVLink message is received
static void handle_parsed_mavlink(uint32_t msg_id, const uint8_t *payload, uint8_t len) {
    if (msg_id == 33) { // GLOBAL_POSITION_INT
        if (len < 28) return;

        // Extract lat, lon, alt from payload
        int32_t lat_int, lon_int, alt_int;
        memcpy(&lat_int, &payload[4], 4);
        memcpy(&lon_int, &payload[8], 4);
        memcpy(&alt_int, &payload[16], 4); // relative alt to home is at offset 16, alt (AMSL) at 12

        float lat = lat_int / 1e7f;
        float lon = lon_int / 1e7f;
        float rel_alt = alt_int / 1000.0f; // mm to meters

        // If home is not set, initialize it with the first valid GPS position
        if (!g_config.home_set) {
            tracker_set_home(lat, lon, 0.0f); // set relative alt home to 0.0
        }

        uint16_t az_us, el_us;
        tracker_calculate_angles(lat, lon, rel_alt, &az_us, &el_us);
        tracker_update_pwm(az_us, el_us);
    }
}

// Parses streaming MAVLink bytes and performs strict X.25 checksum validation
void tracker_parse_mavlink_byte(uint8_t byte) {
    switch (s_mav_state) {
        case MAV_STATE_UNINIT:
            if (byte == 0xFE) { // MAVLink v1 STX
                s_is_v2 = false;
                s_mav_crc = 0xFFFF;
                s_mav_state = MAV_STATE_GOT_STX;
            } else if (byte == 0xFD) { // MAVLink v2 STX
                s_is_v2 = true;
                s_mav_crc = 0xFFFF;
                s_mav_state = MAV_STATE_GOT_STX;
            }
            break;

        case MAV_STATE_GOT_STX:
            s_mav_len = byte;
            s_mav_crc = mavlink_crc_accumulate(byte, s_mav_crc);
            if (s_is_v2) {
                s_mav_state = MAV_STATE_GOT_INCOMPAT;
            } else {
                s_mav_state = MAV_STATE_GOT_SEQ;
            }
            break;

        case MAV_STATE_GOT_INCOMPAT:
            s_mav_crc = mavlink_crc_accumulate(byte, s_mav_crc);
            s_mav_state = MAV_STATE_GOT_COMPAT;
            break;

        case MAV_STATE_GOT_COMPAT:
            s_mav_crc = mavlink_crc_accumulate(byte, s_mav_crc);
            s_mav_state = MAV_STATE_GOT_SEQ;
            break;

        case MAV_STATE_GOT_SEQ:
            s_mav_seq = byte;
            s_mav_crc = mavlink_crc_accumulate(byte, s_mav_crc);
            s_mav_state = MAV_STATE_GOT_SYSID;
            break;

        case MAV_STATE_GOT_SYSID:
            s_mav_crc = mavlink_crc_accumulate(byte, s_mav_crc);
            s_mav_state = MAV_STATE_GOT_COMPID;
            break;

        case MAV_STATE_GOT_COMPID:
            s_mav_crc = mavlink_crc_accumulate(byte, s_mav_crc);
            if (s_is_v2) {
                s_mav_state = MAV_STATE_GOT_MSGID0;
            } else {
                s_mav_msg_id = byte;
                s_mav_payload_idx = 0;
                s_mav_state = MAV_STATE_PAYLOAD;
            }
            break;

        case MAV_STATE_GOT_MSGID0:
            s_mav_msg_id = byte;
            s_mav_crc = mavlink_crc_accumulate(byte, s_mav_crc);
            s_mav_state = MAV_STATE_GOT_MSGID1;
            break;

        case MAV_STATE_GOT_MSGID1:
            s_mav_msg_id |= (byte << 8);
            s_mav_crc = mavlink_crc_accumulate(byte, s_mav_crc);
            s_mav_state = MAV_STATE_GOT_MSGID2;
            break;

        case MAV_STATE_GOT_MSGID2:
            s_mav_msg_id |= (byte << 16);
            s_mav_crc = mavlink_crc_accumulate(byte, s_mav_crc);
            s_mav_payload_idx = 0;
            if (s_mav_len == 0) {
                s_mav_state = MAV_STATE_CHECKSUM_LSB;
            } else {
                s_mav_state = MAV_STATE_PAYLOAD;
            }
            break;

        case MAV_STATE_PAYLOAD:
            s_mav_payload[s_mav_payload_idx++] = byte;
            s_mav_crc = mavlink_crc_accumulate(byte, s_mav_crc);
            if (s_mav_payload_idx >= s_mav_len) {
                s_mav_state = MAV_STATE_CHECKSUM_LSB;
            }
            break;

        case MAV_STATE_CHECKSUM_LSB:
            s_parsed_crc = byte;
            s_mav_state = MAV_STATE_CHECKSUM_MSB;
            break;

        case MAV_STATE_CHECKSUM_MSB:
            s_parsed_crc |= (byte << 8);
            s_mav_state = MAV_STATE_UNINIT;

            // Add MSG_ID extra byte to CRC verification based on MAVLink specifications
            uint8_t extra_byte = 0;
            if (s_mav_msg_id == 33) {
                extra_byte = 104; // GLOBAL_POSITION_INT CRC Extra
            }
            uint16_t final_crc = mavlink_crc_accumulate(extra_byte, s_mav_crc);

            if (final_crc == s_parsed_crc) {
                handle_parsed_mavlink(s_mav_msg_id, s_mav_payload, s_mav_len);
            }
            break;
    }
}
