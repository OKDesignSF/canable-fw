#include "cobs.h"

static void resetDecoder(cobs_decoder_t* cd)
{
    cd->state = COBS_DECODE_STATE_WAITING_FOR_ZERO_OFFSET;
    cd->buffer_length = 0;
    cd->next_zero_offset = 0;
}

void cobs_initDecoder(cobs_decoder_t* cd, uint8_t* buffer, uint8_t buffer_length_max)
{
    cd->buffer = buffer;
    cd->buffer_length_max = buffer_length_max;

    resetDecoder(cd);
}

uint8_t cobs_decodeNextByte(cobs_decoder_t* cd, uint8_t byte)
{
    if (byte == 0) {
        uint8_t buffer_length = cd->buffer_length;
        resetDecoder(cd);
        return buffer_length;
    }

    if (cd->state == COBS_DECODE_STATE_WAITING_FOR_ZERO_OFFSET) {
        cd->next_zero_offset = byte;
        cd->state = COBS_DECODE_STATE_COLLECTING_DATA;
        return 0;
    }

    if (cd->state == COBS_DECODE_STATE_COLLECTING_DATA) {
        if (cd->buffer_length >= cd->buffer_length_max) {
            resetDecoder(cd);
            return 0;
        }

        cd->next_zero_offset -= 1;

        if (cd->next_zero_offset == 0) {
            cd->next_zero_offset = byte;
            byte = 0;
        }

        cd->buffer[cd->buffer_length] = byte;
        cd->buffer_length++;

        return 0;
    }

    return 0;
}

uint8_t cobs_encode(const uint8_t* buffer, uint32_t buffer_length, uint8_t* buffer_out)
{
    // NOTE: This can only encode up to 256 bytes

    buffer_out[0] = 0;
    uint8_t buffer_out_length = 1;

    uint32_t last_zero_offset_index = 0;
    uint32_t zero_offset = 1;

    for (uint32_t i = 0; i < buffer_length; i++) {
        uint32_t byte = buffer[i];
        buffer_out[buffer_out_length] = byte;
        buffer_out_length++;

        if (byte == 0) {
            buffer_out[last_zero_offset_index] = zero_offset;
            zero_offset = 0;
            last_zero_offset_index = buffer_out_length - 1;
        }

        zero_offset++;
    }

    buffer_out[last_zero_offset_index] = zero_offset;

    buffer_out[buffer_out_length] = 0;
    buffer_out_length++;

    return buffer_out_length;
}
