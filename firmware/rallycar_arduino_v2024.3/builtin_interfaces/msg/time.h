#ifndef _TIME_H_
#define _TIME_H_

#include <stdint.h>
#include <stdio.h>
#include "../../field_type.h"
#include "../../message_serializer.h"

namespace builtin_interfaces__msg {
class Time {
public:
    int32_t     sec;
    uint32_t    nanosec;

    Time() : sec(0), nanosec(0) {}

    Time(const int32_t& s, const uint32_t& ns) : sec(s), nanosec(ns) {
        normalizeSecNSec(sec, nanosec);
    }

    static Time fromMicroSec(const uint32_t& t) {
        return Time((int32_t)(t/1000000UL), (t % 1000000UL) * 1000U);
    }

    static uint32_t toMicroSec (const Time& obj) {
        return obj.sec * 1000000UL + obj.nanosec / 1000U;
    }

    static const char* getTypeName(){ return "builtin_interfaces/msg/Time"; }

    static uint16_t getLayout(uint8_t* outBuffer){
        uint8_t* ptr = outBuffer + 4;                   // first 4 bytes reserved for length + checksum
        ptr += sprintf((char *)ptr, "sec");     ptr++;  // field name
        *ptr = (uint8_t)FieldType::INT32;       ptr++;  // field type
        ptr += sprintf((char *)ptr, "nanosec"); ptr++;  // field name
        *ptr = (uint8_t)FieldType::UINT32;      ptr++;  // field type
        uint16_t crc = get_crc_ccitt_checksum(outBuffer + 4, ptr - outBuffer - 4);
        serialize16b(&crc, outBuffer + 2);              // add checksum of field name/type pairs
        return updateSize16b(outBuffer, ptr);
    }

    static uint16_t serialize(const Time& obj, uint8_t* outBuffer){
        uint8_t* ptr = outBuffer;
        ptr += serialize32b((uint32_t *)&(obj.sec), ptr);
        ptr += serialize32b((uint32_t *)&(obj.nanosec), ptr);
        return (ptr - outBuffer);
    }

    static uint16_t deserialize(Time& obj, const uint8_t* inBuffer){
        const uint8_t * ptr = inBuffer;
        ptr += deserialize32b((uint32_t *)&(obj.sec), ptr);
        ptr += deserialize32b((uint32_t *)&(obj.nanosec), ptr);
        return (ptr - inBuffer);
    }

protected:
    static void normalizeSecNSec(int32_t& sec, uint32_t& nsec)
    {
        uint32_t sec_part = nsec / 1000000000UL;
        if (!sec_part) return;
        uint32_t nsec_part = nsec % 1000000000UL;
        sec += sec_part;
        nsec = nsec_part;
    }
};

}

#endif
