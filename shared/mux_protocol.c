#include "mux_protocol.h"
#include <string.h>

void mux_parser_init(MuxParser *parser) {
    if (!parser) return;
    parser->state = MUX_STATE_SYNC1;
    parser->payload_index = 0;
    parser->calc_checksum = 0;
    memset(&parser->frame, 0, sizeof(MuxFrame));
}

uint8_t mux_calculate_checksum(uint8_t chan_id, uint8_t len, const uint8_t *payload) {
    uint8_t cksum = chan_id + len;
    for (uint8_t i = 0; i < len; i++) {
        cksum += payload[i];
    }
    return cksum;
}

bool mux_parse_byte(MuxParser *parser, uint8_t byte, MuxFrame *out_frame) {
    if (!parser || !out_frame) return false;

    switch (parser->state) {
        case MUX_STATE_SYNC1:
            if (byte == MUX_SYNC1) {
                parser->state = MUX_STATE_SYNC2;
            }
            break;

        case MUX_STATE_SYNC2:
            if (byte == MUX_SYNC2) {
                parser->state = MUX_STATE_CHAN_ID;
            } else if (byte == MUX_SYNC1) {
                // Stay in SYNC2 state if we get another SYNC1
                parser->state = MUX_STATE_SYNC2;
            } else {
                parser->state = MUX_STATE_SYNC1;
            }
            break;

        case MUX_STATE_CHAN_ID:
            if (byte == MUX_CHAN_CRSF || byte == MUX_CHAN_MAVLINK || byte == MUX_CHAN_CONFIG) {
                parser->frame.chan_id = byte;
                parser->state = MUX_STATE_LEN;
            } else if (byte == MUX_SYNC1) {
                parser->state = MUX_STATE_SYNC2;
            } else {
                parser->state = MUX_STATE_SYNC1;
            }
            break;

        case MUX_STATE_LEN:
            parser->frame.len = byte;
            parser->payload_index = 0;
            if (byte == 0) {
                parser->state = MUX_STATE_CHECKSUM;
            } else {
                parser->state = MUX_STATE_PAYLOAD;
            }
            break;

        case MUX_STATE_PAYLOAD:
            parser->frame.payload[parser->payload_index++] = byte;
            if (parser->payload_index >= parser->frame.len || parser->payload_index >= MUX_MAX_PAYLOAD) {
                parser->state = MUX_STATE_CHECKSUM;
            }
            break;

        case MUX_STATE_CHECKSUM:
            parser->frame.checksum = byte;
            parser->calc_checksum = mux_calculate_checksum(parser->frame.chan_id, parser->frame.len, parser->frame.payload);
            parser->state = MUX_STATE_SYNC1; // Always reset to sync state

            if (parser->calc_checksum == parser->frame.checksum) {
                memcpy(out_frame, &parser->frame, sizeof(MuxFrame));
                return true;
            }
            break;
    }

    return false;
}

uint16_t mux_encode(uint8_t chan_id, const uint8_t *payload, uint8_t len, uint8_t *out_buffer) {
    if (!out_buffer) return 0;

    out_buffer[0] = MUX_SYNC1;
    out_buffer[1] = MUX_SYNC2;
    out_buffer[2] = chan_id;
    out_buffer[3] = len;

    if (payload && len > 0) {
        memcpy(&out_buffer[4], payload, len);
    }

    uint8_t cksum = mux_calculate_checksum(chan_id, len, payload);
    out_buffer[4 + len] = cksum;

    return 4 + len + 1;
}
