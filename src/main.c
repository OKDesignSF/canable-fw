//
// CANable firmware
//

#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"

#include "cobs.h"
#include "usb_device.h"
#include "usb_cdc.h"
#include "can.h"
#include "okcan.h"
#include "system.h"
#include "led.h"
#include "error.h"

int main(void)
{
    // Initialize peripherals
    system_init();
    can_init();
    led_init();
    usb_init();
    usb_cdc_init();
    okcan_init();

    led_blue_blink(2);
    can_enable();

    // Storage for status and received message buffer
    CAN_RxHeaderTypeDef rx_msg_header;
    uint8_t rx_msg_data[8] = {0};
    uint8_t msg_buf[OKCAN_MAX_MESSAGE_LENGTH];
    uint8_t cobs_buf[OKCAN_MAX_MESSAGE_LENGTH * 2];

    while(1) {
        usb_cdc_process();
        led_process();
        can_process();

        // If CAN message receive is pending, process the message
        if (is_can_msg_pending(CAN_RX_FIFO0)) {
            // If message received from bus, parse the frame
            if (can_rx(&rx_msg_header, rx_msg_data) == HAL_OK) {
                uint8_t msg_len = okcan_encodeCanFrameForUsb(&rx_msg_header, rx_msg_data, msg_buf);

                // Transmit message via USB-CDC
                if (msg_len) {
                    uint8_t cobs_len = cobs_encode(msg_buf, msg_len, cobs_buf);
                    usb_cdc_sendData(cobs_buf, cobs_len);
                }
            }
        }
    }
}

