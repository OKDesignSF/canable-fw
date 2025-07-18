//
// slcan: Parse incoming and generate outgoing slcan messages
//

#include "stm32f0xx_hal.h"
#include <string.h>
#include "can.h"
#include "error.h"
#include "slcan.h"
#include "printf.h"
#include "usbd_cdc_if.h"


// Parse an incoming CAN frame into an outgoing slcan message
int8_t slcan_parse_frame(uint8_t *buf, CAN_RxHeaderTypeDef *frame_header, uint8_t* frame_data)
{
    uint8_t msg_position = 0;

    for (uint8_t j=0; j < SLCAN_MTU; j++)
    {
        buf[j] = '\0';
    }

    // Add character for frame type
    if (frame_header->RTR == CAN_RTR_DATA)
    {
        buf[msg_position] = 't';
    } else if (frame_header->RTR == CAN_RTR_REMOTE) {
        buf[msg_position] = 'r';
    }

    // Assume standard identifier
    uint8_t id_len = SLCAN_STD_ID_LEN;
    uint32_t can_id = frame_header->StdId;

    // Check if extended
    if (frame_header->IDE == CAN_ID_EXT)
    {
        // Convert first char to upper case for extended frame
        buf[msg_position] -= 32;
        id_len = SLCAN_EXT_ID_LEN;
        can_id = frame_header->ExtId;
    }
    msg_position++;

    // Add identifier to buffer
    for(uint8_t j = id_len; j > 0; j--)
    {
        // Add nybble to buffer
        buf[j] = (can_id & 0xF);
        can_id = can_id >> 4;
        msg_position++;
    }

    // Add DLC to buffer
    buf[msg_position++] = frame_header->DLC;

    // Add data bytes
    for (uint8_t j = 0; j < frame_header->DLC; j++)
    {
        buf[msg_position++] = (frame_data[j] >> 4);
        buf[msg_position++] = (frame_data[j] & 0x0F);
    }

    // Convert to ASCII (2nd character to end)
    for (uint8_t j = 1; j < msg_position; j++)
    {
        if (buf[j] < 0xA) {
            buf[j] += 0x30;
        } else {
            buf[j] += 0x37;
        }
    }

    // Add CR (slcan EOL)
    buf[msg_position++] = '\r';

    // Return number of bytes in string
    return msg_position;
}

// Parse an incoming slcan command from the USB CDC port
int8_t slcan_parse_str(uint8_t *buf, uint8_t len)
{
	CAN_TxHeaderTypeDef frame_header;

	// Default to standard ID unless otherwise specified
	frame_header.IDE = CAN_ID_STD;
    frame_header.StdId = 0;
    frame_header.ExtId = 0;

    uint8_t t_cmd_decode_buf[16];
    uint8_t t_cmd_decode_buf_length = 0;

    if (buf[0] == 't') {
        // De-escape
        for (uint8_t i = 1; i < len; i++) {
            // If it's an escape character (0xFF)
            if (buf[i] == 0xFF) {
                if (buf[i + 1] == 0xFE) {
                    // If the next byte is 0xFE, that means the real data is 0xFF
                    t_cmd_decode_buf[t_cmd_decode_buf_length] = 0xFF;
                } else if (buf[i + 1] == 0xFD) {
                    // If the next byte is 0xFD, that means the real data is '\r'
                    t_cmd_decode_buf[t_cmd_decode_buf_length] = '\r';
                }

                i += 1;
            } else {
                t_cmd_decode_buf[t_cmd_decode_buf_length] = buf[i];
            }

            t_cmd_decode_buf_length++;
        }
    } else {
        // Convert from ASCII (2nd character to end)
        if (buf[0] != 't') {
            for (uint8_t i = 1; i < len; i++)
            {
                // Lowercase letters
                if(buf[i] >= 'a')
                    buf[i] = buf[i] - 'a' + 10;
                // Uppercase letters
                else if(buf[i] >= 'A')
                    buf[i] = buf[i] - 'A' + 10;
                // Numbers
                else
                    buf[i] = buf[i] - '0';
            }
        }
    }

    // Process command
    switch(buf[0])
    {
		case 'O':
			// Open channel command
			can_enable();
			return 0;

		case 'C':
			// Close channel command
			can_disable();
			return 0;

		case 'S':
			// Set bitrate command

			// Check for valid bitrate
			if(buf[1] >= CAN_BITRATE_INVALID)
			{
				return -1;
			}

			can_set_bitrate(buf[1]);
			return 0;

		case 'm':
		case 'M':
			// Set mode command
			if (buf[1] == 1)
			{
				// Mode 1: silent
				can_set_silent(1);
			} else {
				// Default to normal mode
				can_set_silent(0);
			}
			return 0;

		case 'a':
		case 'A':
			// Set autoretry command
			if (buf[1] == 1)
			{
				// Mode 1: autoretry enabled (default)
				can_set_autoretransmit(1);
			} else {
				// Mode 0: autoretry disabled
				can_set_autoretransmit(0);
			}
			return 0;

		case 'V':
		{
			// Report firmware version and remote
			char* fw_id = GIT_VERSION " " GIT_REMOTE "\r";
			CDC_Transmit_FS((uint8_t*)fw_id, strlen(fw_id));
			return 0;
		}

	    // Nonstandard!
		case 'E':
		{
	        // Report error register
			char errstr[64] = {0};
			snprintf_(errstr, 64, "CANable Error Register: %X", (unsigned int)error_reg());
			CDC_Transmit_FS((uint8_t*)errstr, strlen(errstr));
	        return 0;
		}

		case 'T':
	    	frame_header.IDE = CAN_ID_EXT;
		case 't':
			// Transmit data frame command
			frame_header.RTR = CAN_RTR_DATA;
			break;

		case 'R':
	    	frame_header.IDE = CAN_ID_EXT;
		case 'r':
			// Transmit remote frame command
			frame_header.RTR = CAN_RTR_REMOTE;
			break;

    	default:
    		// Error, unknown command
    		return -1;
    }


    // Save CAN ID depending on ID type
    uint8_t msg_position = 1;
    if (frame_header.IDE == CAN_ID_EXT) {
        while (msg_position <= SLCAN_EXT_ID_LEN) {
        	frame_header.ExtId *= 16;
        	frame_header.ExtId += buf[msg_position++];
        }
    }
    else {
        if (buf[0] == 't') {
            msg_position = 0;
            frame_header.StdId = t_cmd_decode_buf[msg_position + 0];
            frame_header.StdId <<= 8;
            frame_header.StdId |= t_cmd_decode_buf[msg_position + 1];
            msg_position += 2;
        } else {
            while (msg_position <= SLCAN_STD_ID_LEN) {
                frame_header.StdId *= 16;
                frame_header.StdId += buf[msg_position++];
            }
        }
    }


    // Attempt to parse DLC and check sanity
    if (buf[0] == 't') {
        frame_header.DLC = t_cmd_decode_buf[msg_position++];
    } else {
        frame_header.DLC = buf[msg_position++];
    }
    if (frame_header.DLC > 8) {
        return -1;
    }

    // Copy frame data to buffer
    uint8_t frame_data[8] = {0};
    for (uint8_t j = 0; j < frame_header.DLC; j++) {
        if (buf[0] == 't') {
            frame_data[j] = t_cmd_decode_buf[msg_position];
            msg_position++;
        } else {
            frame_data[j] = (buf[msg_position] << 4) + buf[msg_position+1];
            msg_position += 2;
        }
    }

    // Transmit the message
    can_tx(&frame_header, frame_data);

    return 0;
}

