#ifndef _RALLYCAR_MSGS_
#define _RALLYCAR_MSGS_

#include <stdio.h>
#include <stdint.h>
#include "message_serializer.h"

class Vector3f {
public:
    float       x;
    float       y;
    float       z;

    static const char* getTypeName(){ return "rallycar_msgs/msg/Vector3f"; }

    static uint16_t getLayout(uint8_t* outBuffer){
        uint8_t* ptr = outBuffer + 4;               // first 4 bytes reserved for length + checksum
        ptr += sprintf((char *)ptr, "x");   ptr++;  // field name
        *ptr = (uint8_t)FieldType::FLOAT;   ptr++;  // field type
        ptr += sprintf((char *)ptr, "y");   ptr++;  // field name
        *ptr = (uint8_t)FieldType::FLOAT;   ptr++;  // field type
        ptr += sprintf((char *)ptr, "z");   ptr++;  // field name
        *ptr = (uint8_t)FieldType::FLOAT;   ptr++;  // field type
        uint16_t crc = get_crc_ccitt_checksum(outBuffer + 4, ptr - outBuffer - 4);
        serialize16b(&crc, outBuffer + 2);          // add checksum of field name/type pairs
        return updateSize16b(outBuffer, ptr);
    }

    static uint16_t serialize(const Vector3f& obj, uint8_t* outBuffer){
        uint8_t* ptr = outBuffer;
        ptr += serialize32b((uint32_t *)&(obj.x), ptr);
        ptr += serialize32b((uint32_t *)&(obj.y), ptr);
        ptr += serialize32b((uint32_t *)&(obj.z), ptr);
        return (ptr - outBuffer);
    }

    static uint16_t deserialize(Vector3f& obj, uint8_t* inBuffer){
        uint8_t * ptr = inBuffer;
        ptr += deserialize32b((uint32_t *)&(obj.x), ptr);
        ptr += deserialize32b((uint32_t *)&(obj.y), ptr);
        ptr += deserialize32b((uint32_t *)&(obj.z), ptr);
        return (ptr - inBuffer);
    }
};

class Quaternionf {
public:
    float       x;
    float       y;
    float       z;
    float       w;

    static const char* getTypeName(){ return "rallycar_msgs/msg/Quaternionf"; }

    static uint16_t getLayout(uint8_t* outBuffer){
        uint8_t* ptr = outBuffer + 4;               // first 4 bytes reserved for length + checksum
        ptr += sprintf((char *)ptr, "x");   ptr++;  // field name
        *ptr = (uint8_t)FieldType::FLOAT;   ptr++;  // field type
        ptr += sprintf((char *)ptr, "y");   ptr++;  // field name
        *ptr = (uint8_t)FieldType::FLOAT;   ptr++;  // field type
        ptr += sprintf((char *)ptr, "z");   ptr++;  // field name
        *ptr = (uint8_t)FieldType::FLOAT;   ptr++;  // field type
        ptr += sprintf((char *)ptr, "w");   ptr++;  // field name
        *ptr = (uint8_t)FieldType::FLOAT;   ptr++;  // field type
        uint16_t crc = get_crc_ccitt_checksum(outBuffer + 4, ptr - outBuffer - 4);
        serialize16b(&crc, outBuffer + 2);          // add checksum of field name/type pairs
        return updateSize16b(outBuffer, ptr);
    }

    static uint16_t serialize(const Quaternionf& obj, uint8_t* outBuffer){
        uint8_t* ptr = outBuffer;
        ptr += serialize32b((uint32_t *)&(obj.x), ptr);
        ptr += serialize32b((uint32_t *)&(obj.y), ptr);
        ptr += serialize32b((uint32_t *)&(obj.z), ptr);
        ptr += serialize32b((uint32_t *)&(obj.w), ptr);
        return (ptr - outBuffer);
    }

    static uint16_t deserialize(Quaternionf& obj, uint8_t* inBuffer){
        uint8_t * ptr = inBuffer;
        ptr += deserialize32b((uint32_t *)&(obj.x), ptr);
        ptr += deserialize32b((uint32_t *)&(obj.y), ptr);
        ptr += deserialize32b((uint32_t *)&(obj.z), ptr);
        ptr += deserialize32b((uint32_t *)&(obj.w), ptr);
        return (ptr - inBuffer);
    }
};

class ImuRaw {
public:
    Time        stamp;
    Quaternionf orientation;
    Vector3f    linear_acceleration;
    Vector3f    angular_velocity;

    static const char* getTypeName() { return "rallycar_msgs/msg/ImuRaw"; }

    static uint16_t getLayout(uint8_t* outBuffer) {
        uint8_t* ptr = outBuffer + 4;               // first 4 bytes reserved for length
        ptr += sprintf((char *)ptr, "stamp");       ptr++;  // field name
        *ptr = (uint8_t)FieldType::NESTED_TYPE;     ptr++;  // field type
        ptr += Time::getLayout(ptr);                // referenced field type
        ptr += sprintf((char *)ptr, "orientation"); ptr++;  // field name
        *ptr = (uint8_t)FieldType::NESTED_TYPE;     ptr++;  // field type
        ptr += Quaternionf::getLayout(ptr);         // referenced field type
        ptr += sprintf((char *)ptr, "linear_acceleration"); ptr++;  // field name
        *ptr = (uint8_t)FieldType::NESTED_TYPE;     ptr++;  // field type
        ptr += Vector3f::getLayout(ptr);            // referenced field type
        ptr += sprintf((char *)ptr, "angular_velocity");    ptr++;  // field name
        *ptr = (uint8_t)FieldType::NESTED_TYPE;     ptr++;  // field type
        ptr += Vector3f::getLayout(ptr);            // referenced field type
        uint16_t crc = get_crc_ccitt_checksum(outBuffer + 4, ptr - outBuffer - 4);
        serialize16b(&crc, outBuffer + 2);          // add checksum of field name/type pairs
        return updateSize16b(outBuffer, ptr);
    }

    static uint16_t serialize(const ImuRaw& obj, uint8_t* outBuffer) {
        uint8_t* ptr = outBuffer;
        ptr += Time::serialize(obj.stamp, ptr);
        ptr += Quaternionf::serialize(obj.orientation, ptr);
        ptr += Vector3f::serialize(obj.linear_acceleration, ptr);
        ptr += Vector3f::serialize(obj.angular_velocity, ptr);
        return (ptr - outBuffer);
    }

    static uint16_t deserialize(ImuRaw& obj, uint8_t* inBuffer){
        uint8_t * ptr = inBuffer;
        ptr += Time::deserialize(obj.stamp, ptr);
        ptr += Quaternionf::deserialize(obj.orientation, ptr);
        ptr += Vector3f::deserialize(obj.linear_acceleration, ptr);
        ptr += Vector3f::deserialize(obj.angular_velocity, ptr);
        return (ptr - inBuffer);
    }
};

#endif
