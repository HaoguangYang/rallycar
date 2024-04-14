# A very mechanical code-generation tool that translates a ROS-style .msg file
# into a header file for structure type description, serialization and
# deserialization. The light-weight ser-des approach is useful for running
# communiation protocol on extremely small MCUs, like the ATMEL328P.
# These MCUs have less than 10kB of memory, which means running even micro-ROS
# on these units are impossibe.

# Assumes the *.msg files are placed under "message_class/msg/" filder, e.g.:
# "std_msgs/msg/Float32.msg", the generated header is "std_msgs/msg/float32.h".
# ONLY SUPPORTS NESTED TYPES FOR NOW. DOES NOT SUPPORT ARRAYS OF ANY KIND.
# Mostly written with the help of ChatGPT.

import os

known_mappings = {
    # .msg types: (C types, RCL type definitions, number of bits)
    "int8":     ("int8_t", "INT8", 8),
    "uint8":    ("uint8_t", "UINT8", 8),
    "int16":    ("int16_t", "INT16", 16),
    "uint16":   ("uint16_t", "UINT16", 16),
    "int32":    ("int32_t", "INT32", 32),
    "uint32":   ("uint32_t", "UINT32", 32),
    "int64":    ("int64_t", "INT64", 64),
    "uint64":   ("uint64_t", "UINT64", 64),
    "float32":  ("float", "FLOAT", 32),
    "float64":  ("double", "DOUBLE", 64),
    "char":     ("char", "CHAR", 8),
    "bool":     ("bool", "BOOL", 8),
    "byte":     ("uint8_t", "BYTE", 8),
}


def find_msg_files(directory):
    msg_files = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(".msg"):
                msg_files.append(os.path.relpath(os.path.join(root, file), directory))
    return msg_files


def msg_typesupport_to_pkg_type(typesupport):
    '''
    Function to split "package" and "msg" parts in a typesupport string, e.g.:
    input: 'std_msgs/Float32'
    output: ('std_msgs', 'Float32')   --> we are using Float32 provided by the std_msgs package
    input: 'float32'
    output: (None, 'float')   --> float is a builtin type
    input: 'Float32'
    output: ('', 'Float32')   --> Float32 is defined inside this package
    '''
    # known mappings
    if typesupport in known_mappings:
        return None, known_mappings[typesupport][0]
    # nested types
    if '/' in typesupport:
        # type definition is from another package
        seg = typesupport.split('/')
        return seg[0], seg[-1]
    else:
        return '', typesupport


def get_array_size_and_type(typesupport):
    '''
    Function to strip array part in a typesupport string, e.g.:
    input: 'float32[]'
    output: (0, 'float32')  --> 0 indicates an unbounded sequence
    input: 'float32'
    output: (-1, 'float32') --> -1 indicates this is not an array/sequence
    input: 'float32[9]'
    output: (9, 'float32')  --> a fixed-size array of 9 elements
    Note: this function does not perform typestring conversion
    '''
    ind1 = typesupport.find('[')
    ind2 = typesupport.find(']')
    if ind1 < 0 or ind2 < 0:
        return -1, typesupport
    if ind2 == ind1 + 1:
        return 0, typesupport[0:ind1]
    return int(typesupport[ind1+1:ind2]), typesupport[0:ind1]


def msg_typesupport_to_header_filename(typesupport):
    incl = ''
    _, realType = get_array_size_and_type(typesupport)
    pkg, msg = msg_typesupport_to_pkg_type(realType)
    # known mappings
    if pkg is None:
        return incl
    # nested types
    incl = msg[0].lower() + \
            ''.join(['_' + char.lower() if char.isupper() else char \
                for char in msg[1:]]) + '.h'
    if len(pkg) == 0:
        # this is a type within the current message package
        incl = './' + incl
    else:
        incl = '../../' + pkg + '/msg/' + incl
    return incl


def unbounded_seq_member_type(pkg, element_type):
    if (pkg is None) or (len(pkg) == 0):
        return 'vector<' + element_type + '>'
    return 'vector<' + pkg + "__msg::" + element_type + '>'


def msg_typesupport_to_member_type(typesupport):
    arraySz, realType = get_array_size_and_type(typesupport)
    pkg, msg = msg_typesupport_to_pkg_type(realType)
    if (arraySz == 0):
        realType = unbounded_seq_member_type(pkg, msg)
    elif (pkg is None) or (len(pkg) == 0) :
        realType = msg
    else:
        realType = pkg + "__msg::" + msg
    return realType, arraySz


def get_field_type(typesupport, size):
    field_type = "NOT_SET"
    if typesupport in known_mappings:
        field_type = known_mappings[typesupport][1]
    else:
        field_type = "NESTED_TYPE"
    if (size == 0):
        field_type = field_type + "_UNBOUNDED_SEQUENCE"
    elif (size > 0):
        field_type = field_type + "_ARRAY"
    return "FieldType::" + field_type


def get_field_bits(typesupport):
    if typesupport in known_mappings:
        return known_mappings[typesupport][2]
    else:
        return -1


def rosmsg_interface_generation_content(h_file, input_file_path):
    '''
    Codegen step 2: determining body
    '''
    # Extracting filename (excluding extension) and directory structure
    class_name = os.path.splitext(os.path.basename(input_file_path))[0]
    directory_structure = os.path.relpath(os.path.dirname(input_file_path), os.getcwd())
    # __ instead of :: to prevent namespace collision with RCL
    namespace = "__".join(directory_structure.split(os.path.sep))

    # 2a: header inclusion
    h_file.write(f'\
#include <stdio.h>\n\
#include <stdint.h>\n\
#include "../../transport_layer/message_serializer.h"\n')

    # Open input file and read lines
    with open(input_file_path, 'r') as msg_file:
        raw_lines = msg_file.readlines()
        lines = [];
        for line in raw_lines:
            # Parse variable type and name from each line
            lines.append(line.strip().split())
        # header inclusion: dependent nested types
        incl_set = set()
        for line in lines:
            variable_type = line[0]
            incl = msg_typesupport_to_header_filename(variable_type)
            if len(incl):
                if incl not in incl_set:
                    h_file.write(f'\
#include "{incl}"\n')
                    incl_set.add(incl)
        # 2b: Write class declaration with filename as class name and namespace as namespace
        h_file.write(f'\
namespace {namespace} {{\n')
        h_file.write(f'\
class {class_name} {{\n')
        h_file.write('public:\n')
        for line in lines:
            variable_type, variable_name = line
            member_type, sz = msg_typesupport_to_member_type(variable_type)
            if sz > 0:
                variable_name = variable_name + '[' + str(sz) + ']'
            # 2c: Write public member variable
            h_file.write(f'    {member_type} {variable_name};\n')

        # 2d: type name function for topic negotiation
        h_file.write(f'\
    static const char* getTypeName() {{\n\
        return "{os.path.splitext(input_file_path)[0]}";\n\
    }}\n')

        # 2e: get layout function for topic negotiation
        h_file.write(f'\
    static uint16_t getLayout(uint8_t* outBuffer) {{\n\
        // first 4 bytes reserved for length + checksum\n\
        uint8_t* ptr = outBuffer + 4;\n')
        for line in lines:
            variable_type, variable_name = line
            h_file.write(f'\
        ptr += sprintf((char *)ptr, "{variable_name}"); ptr ++;\n')
            sz, variable_type = get_array_size_and_type(variable_type)
            field_type = get_field_type(variable_type, sz)
            h_file.write(f'\
        *ptr = (uint8_t){field_type}; ptr++;\n')
            if sz > 0:
                h_file.write(f'\
        const uint16_t array_size = {str(sz)};\n\
        ptr += serialize16b(&array_size, ptr);\n')
            if 'NESTED_TYPE' in field_type:
                variable_type, _ = msg_typesupport_to_member_type(variable_type)
                h_file.write(f'\
        ptr += {variable_type}::getLayout(ptr);\n')
        h_file.write(f'\
        uint16_t crc = get_crc_ccitt_checksum(outBuffer + 4, ptr - outBuffer - 4);\n\
        // add checksum of field name/type pairs\n\
        serialize16b(&crc, outBuffer + 2);\n\
        return updateSize16b(outBuffer, ptr);\n\
    }}\n')

        # 2f: serialization function
        h_file.write(f'\
    static uint16_t serialize(const {class_name}& obj, uint8_t* outBuffer) {{\n\
        uint8_t* ptr = outBuffer;\n')
        for line in lines:
            variable_type, variable_name = line
            sz, variable_type = get_array_size_and_type(variable_type)
            if sz > 0:
                # array type, put serialization into a for loop
                h_file.write(f'\
        for (uint16_t i = 0; i < sizeof(obj.{variable_name})/sizeof({variable_type}); i ++) {{\n')
                bits = get_field_bits(variable_type)
                if bits > 0:
                    h_file.write(f'\
            ptr += serialize{bits}b((uint{bits}_t *)&(obj.{variable_name}[i]), ptr);\n')
                else:
                    # this is a nested type
                    variable_type, _ = msg_typesupport_to_member_type(variable_type)
                    h_file.write(f'\
            ptr += {variable_type}::serialize(obj.{variable_name}[i], ptr);\n')
                h_file.write(f'\
        }}\n')
            elif sz < 0:
                # single field
                bits = get_field_bits(variable_type)
                if bits > 0:
                    h_file.write(f'\
        ptr += serialize{bits}b((uint{bits}_t *)&(obj.{variable_name}), ptr);\n')
                else:
                    # this is a nested type
                    variable_type, _ = msg_typesupport_to_member_type(variable_type)
                    h_file.write(f'\
        ptr += {variable_type}::serialize(obj.{variable_name}, ptr);\n')
            else:
                # unbounded sequence
                h_file.write(f'\
        ptr += {unbounded_seq_member_type(None, variable_type)}::serialize(obj.{variable_name}, ptr);\n')
        h_file.write(f'\
        return (ptr - outBuffer);\n\
    }}\n')

        # 2g: deserialization function
        h_file.write(f'\
    static uint16_t deserialize({class_name}& obj, const uint8_t* inBuffer) {{\n\
        const uint8_t* ptr = inBuffer;\n')
        for line in lines:
            variable_type, variable_name = line
            sz, variable_type = get_array_size_and_type(variable_type)
            if sz > 0:
                # array type, put serialization into a for loop
                h_file.write(f'\
        for (uint16_t i = 0; i < sizeof(obj.{variable_name})/sizeof({variable_type}); i ++) {{\n')
                bits = get_field_bits(variable_type)
                if bits > 0:
                    h_file.write(f'\
            ptr += deserialize{bits}b((uint{bits}_t *)&(obj.{variable_name}[i]), ptr);\n')
                else:
                    # this is a nested type
                    variable_type, _ = msg_typesupport_to_member_type(variable_type)
                    h_file.write(f'\
            ptr += {variable_type}::deserialize(obj.{variable_name}[i], ptr);\n')
                h_file.write(f'\
        }}\n')
            elif sz < 0:
                # single field
                bits = get_field_bits(variable_type)
                if bits > 0:
                    h_file.write(f'\
        ptr += deserialize{bits}b((uint{bits}_t *)&(obj.{variable_name}), ptr);\n')
                else:
                    # this is a nested type
                    variable_type, _ = msg_typesupport_to_member_type(variable_type)
                    h_file.write(f'\
        ptr += {variable_type}::deserialize(obj.{variable_name}, ptr);\n')
            else:
                # unbounded sequence
                h_file.write(f'\
        ptr += {unbounded_seq_member_type(None, variable_type)}::deserialize(obj.{variable_name}, ptr);\n')
        h_file.write(f'\
        return (ptr - inBuffer);\n\
    }}\n')

    # 2h: Close class and namespace
    h_file.write('};\n')
    h_file.write('}\n')


def rosmsg_interface_generation(input_file_path):
    '''
    Codegen step 1: determining file location and head+tail formatting
    '''
    # Extracting filename and extension
    file_name, file_extension = os.path.splitext(input_file_path)

    # Replacing uppercase letters with underscore followed by lowercase letters
    new_file_name = ''
    skip_underscore = True  # Flag to skip adding underscore before the first uppercase letter after '/'
    for char in file_name:
        if char.isupper() and not skip_underscore:
            new_file_name += '_' + char.lower()
        elif char == '/':
            new_file_name += char
            skip_underscore = True  # Set the flag to False after encountering '/'
        else:
            new_file_name += char.lower()
            skip_underscore = False
    if new_file_name.startswith('_'):
        new_file_name = new_file_name[1:]  # Remove leading underscore if any

    # Generating new file path
    new_file_path = new_file_name + '.h'

    # Creating the new .h file with content
    with open(new_file_path, 'w') as h_file:
        # Updated pattern for #ifndef guard
        ifndef_guard = '__' + new_file_name.upper() + '_H__'
        ifndef_guard = ifndef_guard.replace('/', '_')  # Replace '/' with underscore
        h_file.write(f'\
// Header file generated from {os.path.basename(input_file_path)}\n\n\
#ifndef {ifndef_guard}\n\
#define {ifndef_guard}\n\n')
        rosmsg_interface_generation_content(h_file, input_file_path)
        h_file.write(f'\
#endif // {ifndef_guard}\n')
    return new_file_path


if __name__ == "__main__":
    current_directory = os.getcwd()
    msg_files = find_msg_files(current_directory)
    if msg_files:
        for msg_file in msg_files:
            rosmsg_interface_generation(msg_file)
