#ifndef _COBS_H
#define _COBS_H

#include <stdint.h>

typedef enum {
    COBS_DECODE_STATE_WAITING_FOR_ZERO_OFFSET = 0,
    COBS_DECODE_STATE_COLLECTING_DATA
} cobs_decode_state_t;

typedef struct {
    cobs_decode_state_t state;
    uint8_t* buffer;
    uint8_t buffer_length_max;
    uint8_t buffer_length;
    uint8_t next_zero_offset;
} cobs_decoder_t;

void cobs_initDecoder(cobs_decoder_t* cd, uint8_t* buffer, uint8_t buffer_length_max);
uint8_t cobs_decodeNextByte(cobs_decoder_t* cd, uint8_t byte);

uint8_t cobs_encode(const uint8_t* buffer, uint32_t buffer_length, uint8_t* buffer_out);

#endif // _COBS_H
