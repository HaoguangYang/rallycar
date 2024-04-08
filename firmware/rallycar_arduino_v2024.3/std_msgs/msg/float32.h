// Header file generated from Float32.msg

#ifndef __STD_MSGS_MSG_FLOAT32_H__
#define __STD_MSGS_MSG_FLOAT32_H__

#include <stdio.h>
#include <stdint.h>
#include "../../message_serializer.h"
namespace std_msgs__msg {
class Float32 {
public:
    float data;
    static const char* getTypeName() {
        return "std_msgs/msg/Float32";
    }
    static uint16_t getLayout(uint8_t* outBuffer) {
        // first 4 bytes reserved for length + checksum
        uint8_t* ptr = outBuffer + 4;
        ptr += sprintf((char *)ptr, "data"); ptr ++;
        *ptr = (uint8_t)FieldType::FLOAT; ptr++;
        uint16_t crc = get_crc_ccitt_checksum(outBuffer + 4, ptr - outBuffer - 4);
        // add checksum of field name/type pairs
        serialize16b(&crc, outBuffer + 2);
        return updateSize16b(outBuffer, ptr);
    }
    static uint16_t serialize(const Float32& obj, uint8_t* outBuffer) {
        uint8_t* ptr = outBuffer;
        ptr += serialize32b((uint32_t *)&(obj.data), ptr);
        return (ptr - outBuffer);
    }
    static uint16_t deserialize(Float32& obj, const uint8_t* inBuffer) {
        const uint8_t* ptr = inBuffer;
        ptr += deserialize32b((uint32_t *)&(obj.data), ptr);
        return (ptr - inBuffer);
    }
};
}
#endif // __STD_MSGS_MSG_FLOAT32_H__
