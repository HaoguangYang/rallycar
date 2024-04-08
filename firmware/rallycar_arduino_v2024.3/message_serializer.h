#ifndef _MESSAGE_SERIALIZER_H_
#define _MESSAGE_SERIALIZER_H_

#include <stdint.h>
#include "field_type.h"
#include "crc16_utils.h"

uint8_t serialize8b(const uint8_t* in, uint8_t* outBuffer){
    outBuffer[0] = *in;
    return 1;
}

uint8_t serialize16b(const uint16_t* in, uint8_t* outBuffer){
    outBuffer[0] = (uint8_t)((*in) & 0xFF);
    outBuffer[1] = (uint8_t)((*in >> 8) & 0xFF);
    return 2;
}

uint8_t serialize32b(const uint32_t* in, uint8_t* outBuffer){
    outBuffer[0] = (uint8_t)((*in) & 0xFF);
    outBuffer[1] = (uint8_t)((*in >> 8) & 0xFF);
    outBuffer[2] = (uint8_t)((*in >> 16) & 0xFF);
    outBuffer[3] = (uint8_t)((*in >> 24) & 0xFF);
    return 4;
}

uint8_t serialize64b(const uint64_t* in, uint8_t* outBuffer){
    outBuffer[0] = (uint8_t)((*in) & 0xFF);
    outBuffer[1] = (uint8_t)((*in >> 8) & 0xFF);
    outBuffer[2] = (uint8_t)((*in >> 16) & 0xFF);
    outBuffer[3] = (uint8_t)((*in >> 24) & 0xFF);
    outBuffer[4] = (uint8_t)((*in >> 32) & 0xFF);
    outBuffer[5] = (uint8_t)((*in >> 40) & 0xFF);
    outBuffer[6] = (uint8_t)((*in >> 48) & 0xFF);
    outBuffer[7] = (uint8_t)((*in >> 56) & 0xFF);
    return 8;
}

uint8_t deserialize8b(uint8_t* out, const uint8_t* inBuffer){
    *out = inBuffer[0];
    return 1;
}

uint8_t deserialize16b(uint16_t* out, const uint8_t* inBuffer){
    *out = (uint16_t)inBuffer[0] | ((uint16_t)inBuffer[1] << 8);
    return 2;
}

uint8_t deserialize32b(uint32_t* out, const uint8_t* inBuffer){
    *out = (uint32_t)inBuffer[0]
            | ((uint32_t)inBuffer[1] << 8)
            | ((uint32_t)inBuffer[2] << 16)
            | ((uint32_t)inBuffer[3] << 24);
    return 4;
}

uint8_t deserialize64b(uint64_t* out, const uint8_t* inBuffer){
    *out = (uint64_t)inBuffer[0]
            | ((uint64_t)inBuffer[1] << 8)
            | ((uint64_t)inBuffer[2] << 16)
            | ((uint64_t)inBuffer[3] << 24)
            | ((uint64_t)inBuffer[4] << 32)
            | ((uint64_t)inBuffer[5] << 40)
            | ((uint64_t)inBuffer[6] << 48)
            | ((uint64_t)inBuffer[7] << 56);
    return 8;
}

uint16_t updateSize16b(uint8_t* buffer, uint8_t* endPtr){
    uint16_t size = endPtr - buffer;
    serialize16b(&size, buffer);
    return size;
}

#endif
