#include "stp23l.h"

#include <string.h>

enum {
    STP23L_SYNC = 0xAA,
    STP23L_SYNC_BYTES = 4,
    STP23L_HEADER_BYTES = 10,
    STP23L_POINT_BYTES = 15,
    STP23L_POINT_COUNT = 12,
    STP23L_MIN_MM = 30,
    STP23L_MAX_MM = 7500,
};

bool stp23l_receive_byte(Stp23lParser* parser, uint8_t byte, uint16_t* distance_mm) {
    static const uint8_t header[] = { 0xAA, 0xAA, 0xAA, 0xAA, 0x00, 0x02, 0x00, 0x00, 0xB8, 0x00 };
    if(parser->used < STP23L_HEADER_BYTES && byte != header[parser->used]) {
        /* Keep the sync prefix if the stream has more than four 0xAA bytes. */
        if(byte == STP23L_SYNC) {
            parser->used = parser->used == STP23L_SYNC_BYTES ? STP23L_SYNC_BYTES : 1u;
            memcpy(parser->frame, header, parser->used);
        }
        else {
            parser->used = 0u;
        }
        return false;
    }
    parser->frame[parser->used++] = byte;
    if(parser->used < STP23L_FRAME_BYTES) return false;
    parser->used = 0u;

    uint8_t checksum = 0u;
    for(unsigned i = STP23L_SYNC_BYTES; i < STP23L_FRAME_BYTES - 1u; ++i) {
        checksum += parser->frame[i];
    }
    if(checksum != byte) return false;

    const unsigned offset = STP23L_HEADER_BYTES + (STP23L_POINT_COUNT - 1u) * STP23L_POINT_BYTES;
    const uint16_t raw = (uint16_t)(parser->frame[offset] | (parser->frame[offset + 1u] << 8u));
    *distance_mm = raw >= STP23L_MIN_MM && raw <= STP23L_MAX_MM ? raw : 0u;
    return true;
}
