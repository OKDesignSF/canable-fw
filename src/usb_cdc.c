#include "usb_cdc.h"

#include "okcan.h"
#include "ringbuf.h"
#include "usbd_cdc_if.h"

//
// Defines
//

#define TX_RINGBUF_CAPACITY      78
#define USB_TRANSFER_LENGTH_MAX  64

//
// State
//

RINGBUF_DEFINE(tx_ringbuf, TX_RINGBUF_CAPACITY, true);

static bool tx_in_progress;
static uint32_t bytes_being_sent;

// Not totally sure where this is defined
extern USBD_HandleTypeDef hUsbDeviceFS;

//
// Helpers
//

static bool isUsbBusy(void)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    return hcdc->TxState != 0;
}

static void startTransferIfNeeded(void)
{
    if (tx_in_progress) {
        return;
    }

    uint32_t data_length;
    uint8_t *data = ringbuf_contiguousPeek(&tx_ringbuf, &data_length);
    if (data_length == 0) {
        return;
    }

    if (data_length > USB_TRANSFER_LENGTH_MAX) {
        data_length = USB_TRANSFER_LENGTH_MAX;
    }

    CDC_Transmit_FS(data, data_length);

    tx_in_progress = true;
    bytes_being_sent = data_length;
}

//
// Public function definitions
//

void usb_cdc_init(void)
{
    ringbuf_init(&tx_ringbuf);

    tx_in_progress = false;
    bytes_being_sent = 0;
}

void usb_cdc_process(void)
{
    if (tx_in_progress) {
        if (!isUsbBusy()) {
            ringbuf_pop(&tx_ringbuf, bytes_being_sent);
            tx_in_progress = false;
            bytes_being_sent = 0;
        }
    }

    startTransferIfNeeded();
}

void usb_cdc_sendData(const uint8_t* data, uint32_t data_length)
{
    if (ringbuf_bytesFree(&tx_ringbuf) < data_length) {
        return;
    }

    ringbuf_put(&tx_ringbuf, data, data_length);

    startTransferIfNeeded();
}

void usb_cdc_receivedData(const uint8_t* data, uint32_t data_length)
{
    for (uint32_t i = 0; i < data_length; i++) {
        okcan_processUsbByte(data[i]);
    }
}
