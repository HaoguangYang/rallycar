// Header file generated from Vector3f.msg

#ifndef __RALLYCAR_MSGS_MSG_VECTOR3F_H__
#define __RALLYCAR_MSGS_MSG_VECTOR3F_H__

#include <stdio.h>
#include <stdint.h>
#include "../../message_serializer.h"
namespace rallycar_msgs__msg {
class Vector3f {
public:
    float x;
    float y;
    float z;
    static const char* getTypeName() {
        return "rallycar_msgs/msg/Vector3f";
    }
    static uint16_t getLayout(uint8_t* outBuffer) {
        // first 4 bytes reserved for length + checksum
        uint8_t* ptr = outBuffer + 4;
        ptr += sprintf((char *)ptr, "x"); ptr ++;
        *ptr = (uint8_t)FieldType::FLOAT; ptr++;
        ptr += sprintf((char *)ptr, "y"); ptr ++;
        *ptr = (uint8_t)FieldType::FLOAT; ptr++;
        ptr += sprintf((char *)ptr, "z"); ptr ++;
        *ptr = (uint8_t)FieldType::FLOAT; ptr++;
        uint16_t crc = get_crc_ccitt_checksum(outBuffer + 4, ptr - outBuffer - 4);
        // add checksum of field name/type pairs
        serialize16b(&crc, outBuffer + 2);
        return updateSize16b(outBuffer, ptr);
    }
    static uint16_t serialize(const Vector3f& obj, uint8_t* outBuffer) {
        uint8_t* ptr = outBuffer;
        ptr += serialize32b((uint32_t *)&(obj.x), ptr);
        ptr += serialize32b((uint32_t *)&(obj.y), ptr);
        ptr += serialize32b((uint32_t *)&(obj.z), ptr);
        return (ptr - outBuffer);
    }
    static uint16_t deserialize(Vector3f& obj, const uint8_t* inBuffer) {
        const uint8_t* ptr = inBuffer;
        ptr += deserialize32b((uint32_t *)&(obj.x), ptr);
        ptr += deserialize32b((uint32_t *)&(obj.y), ptr);
        ptr += deserialize32b((uint32_t *)&(obj.z), ptr);
        return (ptr - inBuffer);
    }
};
}
#endif // __RALLYCAR_MSGS_MSG_VECTOR3F_H__
