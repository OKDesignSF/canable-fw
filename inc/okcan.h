#ifndef _OKCAN_H
#define _OKCAN_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f0xx_hal.h"

#define OKCAN_MAX_MESSAGE_LENGTH (2 + 8)

void okcan_init(void);
bool okcan_processUsbByte(uint8_t byte);
uint8_t okcan_encodeCanFrameForUsb(CAN_RxHeaderTypeDef* can_header, uint8_t* can_data, uint8_t* usb_frame);

#endif // _OKCAN_H
