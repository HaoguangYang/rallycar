// Header file generated from Float32MultiArray.msg

#ifndef __STD_MSGS_MSG_FLOAT32_MULTI_ARRAY_H__
#define __STD_MSGS_MSG_FLOAT32_MULTI_ARRAY_H__

#include <stdio.h>
#include <stdint.h>
#include "../../transport_layer/message_serializer.h"
namespace std_msgs__msg {
class Float32MultiArray {
public:
    vector<float> data;
    static const char* getTypeName() {
        return "std_msgs/msg/Float32MultiArray";
    }
    static uint16_t getLayout(uint8_t* outBuffer) {
        // first 4 bytes reserved for length + checksum
        uint8_t* ptr = outBuffer + 4;
        ptr += sprintf((char *)ptr, "data"); ptr ++;
        *ptr = (uint8_t)FieldType::FLOAT_UNBOUNDED_SEQUENCE; ptr++;
        uint16_t crc = get_crc_ccitt_checksum(outBuffer + 4, ptr - outBuffer - 4);
        // add checksum of field name/type pairs
        serialize16b(&crc, outBuffer + 2);
        return updateSize16b(outBuffer, ptr);
    }
    static uint16_t serialize(const Float32MultiArray& obj, uint8_t* outBuffer) {
        uint8_t* ptr = outBuffer;
        ptr += vector<float32>::serialize(obj.data, ptr);
        return (ptr - outBuffer);
    }
    static uint16_t deserialize(Float32MultiArray& obj, const uint8_t* inBuffer) {
        const uint8_t* ptr = inBuffer;
        ptr += vector<float32>::deserialize(obj.data, ptr);
        return (ptr - inBuffer);
    }
};
}
#endif // __STD_MSGS_MSG_FLOAT32_MULTI_ARRAY_H__
