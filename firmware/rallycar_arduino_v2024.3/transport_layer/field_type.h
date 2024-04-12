// Represents the type of a field and related meta-data.

// A constant for each type supported according to:
//   http://design.ros2.org/articles/legacy_interface_definition.html
// and:
//   http://design.ros2.org/articles/idl_interface_definition.html
// Order is loosely coupled to the order of appearance in the IDL 4.2 spec:
//  https://www.omg.org/spec/IDL/4.2

// Layout of constants across the 0-255 decimal values in the uint8:
//
// - 000    : Reserved for "not set"
// - 001-048: Primitive types, strings, and reserved space for future primitive types
// - 049-096: Fixed sized array of primitive and string types
// - 097-144: Bounded Sequences of primitive and string types
// - 145-192: Unbounded Sequences of primitive and string types
// - 193-255: Reserved space for future array/sequence-like types

#ifndef _FIELD_TYPE_H_
#define _FIELD_TYPE_H_

#include <stdint.h>

enum class NodeServiceID : uint8_t {
    TIME_SYNC_ID = 0xFD,
    RESET_ID = 0xFE,
    UNASSIGNED = 0xFF
};

enum class EndpointType : uint8_t {
    Invalid = 0,
    Publisher,
    Subscription
};

enum class NodeState : uint8_t {
    OUT_OF_SYNC = 0,
    RUNNING
};

enum class FieldType : uint8_t {
    NOT_SET = 0,

    // Nested type defined in other .msg/.idl files.
    NESTED_TYPE = 1,

    // Integer Types
    INT8 = 2,
    UINT8 = 3,
    INT16 = 4,
    UINT16 = 5,
    INT32 = 6,
    UINT32 = 7,
    INT64 = 8,
    UINT64 = 9,

    // Floating-Point Types
    FLOAT = 10,
    DOUBLE = 11,
    LONG_DOUBLE = 12,

    // Char and WChar Types
    CHAR = 13,
    WCHAR = 14,

    // Boolean Type
    BOOLEAN = 15,

    // Byte/Octet Type
    BYTE = 16,

    // String Types
    STRING = 17,
    WSTRING = 18,

    // Fixed String Types
    FIXED_STRING = 19,
    FIXED_WSTRING = 20,

    // Bounded String Types
    BOUNDED_STRING = 21,
    BOUNDED_WSTRING = 22,

    // Fixed Sized Array Types
    NESTED_TYPE_ARRAY = 49,
    INT8_ARRAY = 50,
    UINT8_ARRAY = 51,
    INT16_ARRAY = 52,
    UINT16_ARRAY = 53,
    INT32_ARRAY = 54,
    UINT32_ARRAY = 55,
    INT64_ARRAY = 56,
    UINT64_ARRAY = 57,
    FLOAT_ARRAY = 58,
    DOUBLE_ARRAY = 59,
    LONG_DOUBLE_ARRAY = 60,
    CHAR_ARRAY = 61,
    WCHAR_ARRAY = 62,
    BOOLEAN_ARRAY = 63,
    BYTE_ARRAY = 64,
    STRING_ARRAY = 65,
    WSTRING_ARRAY = 66,
    FIXED_STRING_ARRAY = 67,
    FIXED_WSTRING_ARRAY = 68,
    BOUNDED_STRING_ARRAY = 69,
    BOUNDED_WSTRING_ARRAY = 70,

    // Bounded Sequence Types
    NESTED_TYPE_BOUNDED_SEQUENCE = 97,
    INT8_BOUNDED_SEQUENCE = 98,
    UINT8_BOUNDED_SEQUENCE = 99,
    INT16_BOUNDED_SEQUENCE = 100,
    UINT16_BOUNDED_SEQUENCE = 101,
    INT32_BOUNDED_SEQUENCE = 102,
    UINT32_BOUNDED_SEQUENCE = 103,
    INT64_BOUNDED_SEQUENCE = 104,
    UINT64_BOUNDED_SEQUENCE = 105,
    FLOAT_BOUNDED_SEQUENCE = 106,
    DOUBLE_BOUNDED_SEQUENCE = 107,
    LONG_DOUBLE_BOUNDED_SEQUENCE = 108,
    CHAR_BOUNDED_SEQUENCE = 109,
    WCHAR_BOUNDED_SEQUENCE = 110,
    BOOLEAN_BOUNDED_SEQUENCE = 111,
    BYTE_BOUNDED_SEQUENCE = 112,
    STRING_BOUNDED_SEQUENCE = 113,
    WSTRING_BOUNDED_SEQUENCE = 114,
    FIXED_STRING_BOUNDED_SEQUENCE = 115,
    FIXED_WSTRING_BOUNDED_SEQUENCE = 116,
    BOUNDED_STRING_BOUNDED_SEQUENCE = 117,
    BOUNDED_WSTRING_BOUNDED_SEQUENCE = 118,

    // Unbounded Sequence Types
    NESTED_TYPE_UNBOUNDED_SEQUENCE = 145,
    INT8_UNBOUNDED_SEQUENCE = 146,
    UINT8_UNBOUNDED_SEQUENCE = 147,
    INT16_UNBOUNDED_SEQUENCE = 148,
    UINT16_UNBOUNDED_SEQUENCE = 149,
    INT32_UNBOUNDED_SEQUENCE = 150,
    UINT32_UNBOUNDED_SEQUENCE = 151,
    INT64_UNBOUNDED_SEQUENCE = 152,
    UINT64_UNBOUNDED_SEQUENCE = 153,
    FLOAT_UNBOUNDED_SEQUENCE = 154,
    DOUBLE_UNBOUNDED_SEQUENCE = 155,
    LONG_DOUBLE_UNBOUNDED_SEQUENCE = 156,
    CHAR_UNBOUNDED_SEQUENCE = 157,
    WCHAR_UNBOUNDED_SEQUENCE = 158,
    BOOLEAN_UNBOUNDED_SEQUENCE = 159,
    BYTE_UNBOUNDED_SEQUENCE = 160,
    STRING_UNBOUNDED_SEQUENCE = 161,
    WSTRING_UNBOUNDED_SEQUENCE = 162,
    FIXED_STRING_UNBOUNDED_SEQUENCE = 163,
    FIXED_WSTRING_UNBOUNDED_SEQUENCE = 164,
    BOUNDED_STRING_UNBOUNDED_SEQUENCE = 165,
    BOUNDED_WSTRING_UNBOUNDED_SEQUENCE = 166
};

#endif
