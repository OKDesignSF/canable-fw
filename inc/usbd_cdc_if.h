#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#include "usbd_cdc.h"

#define TX_BUF_SIZE 64
#define RX_BUF_SIZE 64

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

#endif // __USBD_CDC_IF_H
