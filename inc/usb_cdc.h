#ifndef __USB_CDC_APP_H
#define __USB_CDC_APP_H

#include <stdbool.h>
#include <stdint.h>

void usb_cdc_init(void);
void usb_cdc_process(void);

void usb_cdc_sendData(const uint8_t* data, uint32_t data_length);

void usb_cdc_receivedData(const uint8_t* data, uint32_t data_length);
bool usb_cdc_hasData(void);
uint8_t usb_cdc_getNextByte(void);

#endif // __USB_CDC_APP_H
