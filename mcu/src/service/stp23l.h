#ifndef STP23L_H
#define STP23L_H

#include <stdbool.h>
#include <stdint.h>

enum { STP23L_FRAME_BYTES = 195 };

typedef struct {
    uint8_t frame[STP23L_FRAME_BYTES];
    uint16_t used;
} Stp23lParser;

/* A complete distance frame yields its newest point in mm, or zero for no valid range. */
bool stp23l_receive_byte(Stp23lParser* parser, uint8_t byte, uint16_t* distance_mm);

#endif
