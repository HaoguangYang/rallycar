// Header file generated from ImuRaw.msg

#ifndef __RALLYCAR_MSGS_MSG_IMU_RAW_H__
#define __RALLYCAR_MSGS_MSG_IMU_RAW_H__

#include <stdio.h>
#include <stdint.h>
#include "../../transport_layer/message_serializer.h"
#include "../../builtin_interfaces/msg/time.h"
#include "./quaternionf.h"
#include "./vector3f.h"
namespace rallycar_msgs__msg {
class ImuRaw {
public:
    builtin_interfaces__msg::Time stamp;
    Quaternionf orientation;
    Vector3f linear_acceleration;
    Vector3f angular_velocity;
    static const char* getTypeName() {
        return "rallycar_msgs/msg/ImuRaw";
    }
    static uint16_t getLayout(uint8_t* outBuffer) {
        // first 4 bytes reserved for length + checksum
        uint8_t* ptr = outBuffer + 4;
        ptr += sprintf((char *)ptr, "stamp"); ptr ++;
        *ptr = (uint8_t)FieldType::NESTED_TYPE; ptr++;
        ptr += builtin_interfaces__msg::Time::getLayout(ptr);
        ptr += sprintf((char *)ptr, "orientation"); ptr ++;
        *ptr = (uint8_t)FieldType::NESTED_TYPE; ptr++;
        ptr += Quaternionf::getLayout(ptr);
        ptr += sprintf((char *)ptr, "linear_acceleration"); ptr ++;
        *ptr = (uint8_t)FieldType::NESTED_TYPE; ptr++;
        ptr += Vector3f::getLayout(ptr);
        ptr += sprintf((char *)ptr, "angular_velocity"); ptr ++;
        *ptr = (uint8_t)FieldType::NESTED_TYPE; ptr++;
        ptr += Vector3f::getLayout(ptr);
        uint16_t crc = get_crc_ccitt_checksum(outBuffer + 4, ptr - outBuffer - 4);
        // add checksum of field name/type pairs
        serialize16b(&crc, outBuffer + 2);
        return updateSize16b(outBuffer, ptr);
    }
    static uint16_t serialize(const ImuRaw& obj, uint8_t* outBuffer) {
        uint8_t* ptr = outBuffer;
        ptr += builtin_interfaces__msg::Time::serialize(obj.stamp, ptr);
        ptr += Quaternionf::serialize(obj.orientation, ptr);
        ptr += Vector3f::serialize(obj.linear_acceleration, ptr);
        ptr += Vector3f::serialize(obj.angular_velocity, ptr);
        return (ptr - outBuffer);
    }
    static uint16_t deserialize(ImuRaw& obj, const uint8_t* inBuffer) {
        const uint8_t* ptr = inBuffer;
        ptr += builtin_interfaces__msg::Time::deserialize(obj.stamp, ptr);
        ptr += Quaternionf::deserialize(obj.orientation, ptr);
        ptr += Vector3f::deserialize(obj.linear_acceleration, ptr);
        ptr += Vector3f::deserialize(obj.angular_velocity, ptr);
        return (ptr - inBuffer);
    }
};
}
#endif // __RALLYCAR_MSGS_MSG_IMU_RAW_H__
