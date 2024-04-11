/*
A minimal HDLC library, which can be used over any interface.
Copyright (C) 2015 Jarkko Hautakorpi et al. see LICENSE.txt

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#ifndef _HDLC_H_
#define _HDLC_H_

#include <stdint.h>
#include <stdbool.h>
#include "crc16_utils.h"

/* HDLC Asynchronous framing */
/* The frame boundary octet is 01111110, (7E in hexadecimal notation) */
#define FRAME_BOUNDARY_OCTET 0x7E

/* A "control escape octet", has the bit sequence '01111101', (7D hexadecimal) */
#define CONTROL_ESCAPE_OCTET 0x7D

/* If either of these two octets appears in the transmitted data, an escape octet is sent, */
/* followed by the original data octet with bit 5 inverted */
#define INVERT_OCTET 0x20

/* 16bit low and high bytes copier */
#define low(x)    ((x) & 0xFF)
#define high(x)   (((x)>>8) & 0xFF)

class HdlcBase {
    public:
    HdlcBase (uint16_t mtu, uint8_t* inBuffer) :
        receive_frame_buffer(inBuffer),
        frame_position(0),
        frame_checksum(CRC16_CCITT_INIT_VAL),
        mtu(mtu),
        escape_character(false)
    {}

    /* Function to find valid HDLC frame from incoming data */
    void receiveChar(uint8_t data) {
        /* FRAME FLAG */
        if(data == FRAME_BOUNDARY_OCTET)
        {
            if(this->escape_character == true)
            {
                this->escape_character = false;
            }
            /* If a valid frame is detected */
            else if ( (this->frame_position >= 2) &&
                      ( this->frame_checksum == (
                        (uint16_t)(this->receive_frame_buffer[this->frame_position-1] << 8 ) |
                        (this->receive_frame_buffer[this->frame_position-2] & 0xff))
                      )
                    )  // (msb << 8 ) | (lsb & 0xff)
            {
                /* Call the user defined function and pass frame to it */
                handle_frame(receive_frame_buffer, (uint16_t)(this->frame_position-2));
            }
            this->frame_position = 0;
            this->frame_checksum = CRC16_CCITT_INIT_VAL;
            return;
        }

        if(this->escape_character)
        {
            this->escape_character = false;
            data ^= INVERT_OCTET;
        }
        else if(data == CONTROL_ESCAPE_OCTET)
        {
            this->escape_character = true;
            return;
        }

        receive_frame_buffer[this->frame_position] = data;

        if(this->frame_position >= 2) {
            this->frame_checksum = _crc_ccitt_update(this->frame_checksum, receive_frame_buffer[this->frame_position-2]);
        }

        this->frame_position++;

        if(this->frame_position == this->mtu)
        {
            this->frame_position = 0;
            this->frame_checksum = CRC16_CCITT_INIT_VAL;
        }
    }

    /* Wrap given data in HDLC frame and send it out byte at a time*/
    void sendFrame(const char *outBuffer, uint16_t frame_length) {
        uint8_t data;
        uint16_t fcs = CRC16_CCITT_INIT_VAL;

        sendchar((uint8_t)FRAME_BOUNDARY_OCTET);

        while(frame_length)
        {
            data = *outBuffer++;
            fcs = _crc_ccitt_update(fcs, data);
            if((data == CONTROL_ESCAPE_OCTET) || (data == FRAME_BOUNDARY_OCTET))
            {
                sendchar((uint8_t)CONTROL_ESCAPE_OCTET);
                data ^= INVERT_OCTET;
            }
            sendchar((uint8_t)data);
            frame_length--;
        }
        data = low(fcs);
        if((data == CONTROL_ESCAPE_OCTET) || (data == FRAME_BOUNDARY_OCTET))
        {
            sendchar((uint8_t)CONTROL_ESCAPE_OCTET);
            data ^= (uint8_t)INVERT_OCTET;
        }
        sendchar((uint8_t)data);
        data = high(fcs);
        if((data == CONTROL_ESCAPE_OCTET) || (data == FRAME_BOUNDARY_OCTET))
        {
            sendchar(CONTROL_ESCAPE_OCTET);
            data ^= INVERT_OCTET;
        }
        sendchar(data);
        sendchar(FRAME_BOUNDARY_OCTET);
    }

  protected:
    virtual void sendchar(const uint8_t& data) = 0;
    virtual void handle_frame(const uint8_t *framebuffer, const uint16_t& framelength) = 0;

  private:
    uint8_t * receive_frame_buffer;
    uint16_t frame_position;
    uint16_t frame_checksum;  // 16bit CRC sum for _crc_ccitt_update
    uint16_t mtu;
    bool escape_character;
};

template <typename InterfaceT>
class Hdlc : public HdlcBase
{
typedef void (InterfaceT::*sendchar_type) (const uint8_t&);
typedef void (InterfaceT::*frame_handler_type)(const uint8_t *framebuffer, const uint16_t& framelength);
private:
    /* User must define a function, that sends a 8bit char over the chosen interface, usart, spi, i2c etc. */
    sendchar_type sendchar_function;
    /* User must define a function, that will process the valid received frame */
    /* This function can act like a command router/dispatcher */
    frame_handler_type frame_handler;

    InterfaceT* intf_;

    /* Function to send a byte throug USART, I2C, SPI etc.*/
    virtual void sendchar(const uint8_t& data) override final {
        (intf_->*sendchar_function)(data);
    }

    virtual void handle_frame(const uint8_t *framebuffer, const uint16_t& framelength) override final {
        (intf_->*frame_handler)(framebuffer, framelength);
    }

  public:
    Hdlc (
        sendchar_type put_char,
        frame_handler_type recvFrameFunctor,
        uint16_t mtu,
        uint8_t* inBuffer,
        InterfaceT* intf
    ) :
        HdlcBase(mtu, inBuffer),
        sendchar_function(put_char),
        frame_handler(recvFrameFunctor),
        intf_(intf)
    {}
};

#endif
