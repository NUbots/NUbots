#!/usr/bin/env python3

import os
import re
import sys

from google.protobuf.descriptor_pb2 import FieldOptions, FileDescriptorSet

# Add the NEUTRON_BUILTIN_DIR to the path so we can import the generated builtin protobufs
# And extend our options with our Neutron protobuf
# If this environment variable is not set we default to our cwd
# The "builtin" protobufs are the ones found in nuclear/message/proto so NEUTRON_BUILTIN_DIR should be set to path
# where the generated python protobuf files are stored
builtin_dir = os.environ.get("NEUTRON_BUILTIN_DIR", os.path.join(os.getcwd(), ".."))
sys.path.append(os.path.join(builtin_dir, "python"))
from Neutron_pb2 import PointerType, array_size, pointer  # isort:skip

PointerType = dict(PointerType.items())


class Field:

    # Static map_types field
    map_types = {}

    def __init__(self, f, context):
        self.name = f.name
        self.number = f.number

        # Some booleans to describe the type
        self.map_type = f.type_name in Field.map_types
        self.repeated = f.label == f.LABEL_REPEATED
        self.pointer = f.options.Extensions[pointer]
        self.array_size = f.options.Extensions[array_size]
        self.bytes_type = f.type == f.TYPE_BYTES
        self.one_of = False

        # Basic types are treated as primitives by the library
        self.basic = f.type not in [f.TYPE_MESSAGE, f.TYPE_GROUP, f.TYPE_BYTES]

        # Map types are special and a little more difficult to spot
        if f.type_name in Field.map_types:
            self.type = Field.map_types[f.type_name]

        # Normal message types
        elif f.type in [f.TYPE_MESSAGE, f.TYPE_ENUM, f.TYPE_GROUP]:
            self.type = f.type_name
            self.default_value = f.default_value

        # Protobuf basic types
        else:
            # Work out what primitive type we have
            # and the default default for that field
            type_info = {
                f.TYPE_DOUBLE: ("double", "0.0"),
                f.TYPE_FLOAT: ("float", "0.0f"),
                f.TYPE_INT64: ("int64", "0"),
                f.TYPE_UINT64: ("uint64", "0"),
                f.TYPE_INT32: ("int32", "0"),
                f.TYPE_FIXED64: ("fixed64", "0"),
                f.TYPE_FIXED32: ("fixed32", "0"),
                f.TYPE_BOOL: ("bool", "false"),
                f.TYPE_STRING: ("string", '""'),
                f.TYPE_BYTES: ("bytes", ""),
                f.TYPE_UINT32: ("uint32", "0"),
                f.TYPE_SFIXED32: ("sfixed32", "0"),
                f.TYPE_SFIXED64: ("sfixed64", "0"),
                f.TYPE_SINT32: ("sint32", "0"),
                f.TYPE_SINT64: ("sint64", "0"),
            }[f.type]

            self.type = type_info[0]
            self.default_value = f.default_value if f.default_value else type_info[1]

        if self.type == ".google.protobuf.Timestamp":
            self.default_value = "NUClear::clock::now()"
        elif self.type == ".google.protobuf.Duration":
            self.default_value = "NUClear::clock::duration(0)"

        # If we are repeated or a pointer our default is changed
        if self.repeated:
            self.default_value = ""
        elif self.pointer:
            self.default_value = "nullptr"

        # Since our cpp_type is used a lot, precalculate it
        self.cpp_type, self.special_cpp_type = self.get_cpp_type_info()

    def get_cpp_type_info(self):

        t = self.type

        # We are special unless we are not
        special = True

        vector_regex = re.compile(r"^\.([fiuc]?)vec(\d*)$")
        matrix_regex = re.compile(r"^\.([fiuc]?)mat(\d*)$")
        quaternion_regex = re.compile(r"^\.(f?)quat$")
        isometry_regex = re.compile(r"^\.(f?)iso([23])$")

        # Nothing is trivially copyable unless it is
        self.trivially_copyable = False

        # Check if it is a map field
        if self.map_type:
            t = "::std::map<{}, {}>".format(t[0].cpp_type, t[1].cpp_type)

        # Check for matrix and vector types
        elif vector_regex.match(t):
            r = vector_regex.match(t)
            t = "::message::conversion::math::{}vec{}".format(r.group(1), r.group(2))
        elif matrix_regex.match(t):
            r = matrix_regex.match(t)
            t = "::message::conversion::math::{}mat{}".format(r.group(1), r.group(2))
        elif quaternion_regex.match(t):
            r = quaternion_regex.match(t)
            t = "::message::conversion::math::{}quat".format(r.group(1))
        elif isometry_regex.match(t):
            r = isometry_regex.match(t)
            t = "::message::conversion::math::{}iso{}".format(r.group(1), r.group(2))

        # Timestamps and durations map to real time/duration classes
        elif t == ".google.protobuf.Timestamp":
            t = "::NUClear::clock::time_point"
            self.trivially_copyable = True
        elif t == ".google.protobuf.Duration":
            t = "::NUClear::clock::duration"
            self.trivially_copyable = True

        # Standard types get mapped to their appropriate type
        elif t in ["double", "float", "bool"]:
            # double and float and bool are fine as is
            special = False
            self.trivially_copyable = True
        elif t in ["int64", "sint64", "sfixed64"]:
            t = "int64_t"
            special = False
            self.trivially_copyable = True
        elif t in ["uint64", "fixed64"]:
            t = "uint64_t"
            special = False
            self.trivially_copyable = True
        elif t in ["int32", "sint32", "sfixed32"]:
            t = "int32_t"
            special = False
            self.trivially_copyable = True
        elif t in ["uint32", "fixed32"]:
            t = "uint32_t"
            special = False
            self.trivially_copyable = True
        elif t in ["string"]:
            t = "::std::string"
            special = False
        elif t in ["bytes"]:
            t = "::std::vector<uint8_t>"
        # Otherwise we assume it's a normal type and let it work out its scoping
        else:
            t = "::".join(t.split("."))
            special = False

        # If we are using a pointer type do the manipulation here
        if self.pointer == PointerType["RAW"]:
            t = "{}*".format(t)
            self.trivially_copyable = True
        elif self.pointer == PointerType["SHARED"]:
            t = "::std::shared_ptr<{}>".format(t)
            self.trivially_copyable = True
        elif self.pointer == PointerType["UNIQUE"]:
            t = "::std::unique_ptr<{}>".format(t)
            self.trivially_copyable = True

        # If it's a repeated field, and not a map, it's a vector
        if self.repeated and not self.map_type:
            # If we have a fixed size use std::array instead
            if self.array_size > 0:
                t = "::std::array<{}, {}>".format(t, self.array_size)
                self.trivially_copyable = False
            else:
                t = "::std::vector<{}>".format(t)
                self.trivially_copyable = False

        return t, special

    def generate_cpp_header(self):
        return "{} {}{{{}}};".format(self.cpp_type, self.name, self.default_value if self.type != "string" else "")
