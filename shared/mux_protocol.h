#ifndef MUX_PROTOCOL_H
#define MUX_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#define MUX_SYNC1 0xAA
#define MUX_SYNC2 0x55

// Channel ID Definitions
#define MUX_CHAN_CRSF    0x01
#define MUX_CHAN_MAVLINK 0x02
#define MUX_CHAN_CONFIG  0x03

#define MUX_MAX_PAYLOAD 255

typedef struct {
    uint8_t chan_id;
    uint8_t len;
    uint8_t payload[MUX_MAX_PAYLOAD];
    uint8_t checksum;
} MuxFrame;

typedef enum {
    MUX_STATE_SYNC1 = 0,
    MUX_STATE_SYNC2,
    MUX_STATE_CHAN_ID,
    MUX_STATE_LEN,
    MUX_STATE_PAYLOAD,
    MUX_STATE_CHECKSUM
} MuxParserState;

typedef struct {
    MuxParserState state;
    MuxFrame frame;
    uint16_t payload_index;
    uint8_t calc_checksum;
} MuxParser;

// Function declarations
void mux_parser_init(MuxParser *parser);
bool mux_parse_byte(MuxParser *parser, uint8_t byte, MuxFrame *out_frame);
uint8_t mux_calculate_checksum(uint8_t chan_id, uint8_t len, const uint8_t *payload);
uint16_t mux_encode(uint8_t chan_id, const uint8_t *payload, uint8_t len, uint8_t *out_buffer);

#endif // MUX_PROTOCOL_H
