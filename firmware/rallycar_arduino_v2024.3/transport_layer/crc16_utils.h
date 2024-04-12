#ifndef _CRC16_UTILS_H_
#define _CRC16_UTILS_H_

#include <stdint.h>

#ifdef ARDUINO
#include "Arduino.h"
#include <util/crc16.h>
#else
uint16_t _crc_ccitt_update (uint16_t crc, uint8_t data)
{
    data ^= crc & 0x00FF;
    data ^= data << 4;
    return ((((uint16_t)data << 8) | ((crc & 0xFF00) >> 8)) ^ (uint8_t)(data >> 4)
            ^ ((uint16_t)data << 3));
}
#endif

/* The frame check sequence (FCS) is a 16-bit CRC-CCITT */
/* AVR Libc CRC function is _crc_ccitt_update() */
/* Corresponding CRC function in Qt (www.qt.io) is qChecksum() */
#define CRC16_CCITT_INIT_VAL 0xFFFF

uint16_t get_crc_ccitt_checksum (uint8_t* data, uint16_t length)
{
    uint16_t crc = CRC16_CCITT_INIT_VAL;
    uint8_t d1;
    for (uint16_t n = 0; n < length; n++) {
        d1 = data[n] ^ (crc & 0x00FF);
        d1 ^= d1 << 4;
        crc = ((((uint16_t)d1 << 8) | ((crc & 0xFF00) >> 8)) ^ (uint8_t)(d1 >> 4)
            ^ ((uint16_t)d1 << 3));
    }
    return crc;
}

#endif
