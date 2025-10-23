#include "okcan.h"

#include "can.h"
#include "cobs.h"

static cobs_decoder_t cobs_decoder;
static uint8_t cobs_decode_buffer[OKCAN_MAX_MESSAGE_LENGTH];

// OKCAN USB frame format:
//  - 2-byte header, MSB first:
//    - Bits [15-15]: Reserved
//    - Bits [14-11]: Data length
//    - Bits [10-0]:  Arbitration ID
//  - Data bytes, up to 8

static void processUsbFrame(uint8_t* buffer, uint8_t buffer_length)
{
    if (buffer_length < 2) {
        return;
    }

    // RLLLLAAA AAAAAAAA
    uint8_t data_length = (buffer[0] & 0x78) >> 3;
    uint16_t arbitration_id = ((buffer[0] & 0x07) << 8) | buffer[1];
    uint8_t* data = &buffer[2];

    if (data_length > 8) {
        return;
    }

    CAN_TxHeaderTypeDef can_header;
    can_header.IDE = CAN_ID_STD;
    can_header.RTR = CAN_RTR_DATA;
    can_header.ExtId = 0;
    can_header.StdId = arbitration_id;
    can_header.DLC = data_length;

    can_tx(&can_header, data);
}

void okcan_init(void)
{
    cobs_initDecoder(&cobs_decoder, cobs_decode_buffer, OKCAN_MAX_MESSAGE_LENGTH);
}

bool okcan_processUsbByte(uint8_t byte)
{
    uint8_t usb_frame_length = cobs_decodeNextByte(&cobs_decoder, byte);
    if (usb_frame_length > 0) {
        processUsbFrame(cobs_decode_buffer, usb_frame_length);
        return true;
    }

    return false;
}

uint8_t okcan_encodeCanFrameForUsb(CAN_RxHeaderTypeDef* can_header, uint8_t* can_data, uint8_t* usb_frame)
{
    uint8_t data_length = can_header->DLC;
    if (data_length > 8) {
        return 0;
    }

    uint16_t arbitration_id = can_header->StdId;

    // RLLLLAAA AAAAAAAA
    uint8_t header_upper = (data_length & 0x0F) << 3 | ((arbitration_id >> 8) & 0x07);
    uint8_t header_lower = arbitration_id & 0xFF;

    usb_frame[0] = header_upper;
    usb_frame[1] = header_lower;

    for (uint8_t i = 0; i < data_length; i++) {
        usb_frame[i + 2] = can_data[i];
    }

    return (data_length + 2);
}
