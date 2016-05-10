#!/usr/bin/python
import re
import os
import sys
import textwrap
import datetime
from google.protobuf.descriptor_pb2 import FileDescriptorSet, FieldOptions

from pygments import highlight
from pygments.lexers import CppLexer
from pygments.formatters import Terminal256Formatter

# Add our cwd to the path so we can import generated python protobufs
# And extend our options with our MessageOptions
sys.path.append(os.getcwd())
from MessageOptions_pb2 import pointer, PointerType
FieldOptions.RegisterExtension(pointer)
PointerType = dict(PointerType.items())


def indent(str, len=4):
    return '\n'.join([(' ' * len) + l for l in str.splitlines()])


def to_camel_case(snake_str):
    components = snake_str.split('_')
    # We capitalize the first letter of each component except the first one
    # with the 'title' method and join them together.
    return components[0] + "".join(x.title() for x in components[1:])


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
        self.bytes_type = f.type == f.TYPE_BYTES;
        # Basic types are treated as primitives by the library
        self.basic = f.type not in [f.TYPE_MESSAGE, f.TYPE_GROUP, f.TYPE_BYTES]

        # Map types are special and a little more difficult to spot
        if f.type_name in Field.map_types:
            self.type = Field.map_types[f.type_name]

        # Normal message types
        elif f.type in [ f.TYPE_MESSAGE, f.TYPE_ENUM, f.TYPE_GROUP ]:
            self.type = f.type_name
            self.default_value = f.default_value

        # Protobuf basic types
        else:
            # Work out what primitive type we have
            # and the default default for that field
            type_info = {
                f.TYPE_DOUBLE:   ('double',   '0.0'),
                f.TYPE_FLOAT:    ('float',    '0.0'),
                f.TYPE_INT64:    ('int64',    '0'),
                f.TYPE_UINT64:   ('uint64',   '0'),
                f.TYPE_INT32:    ('int32',    '0'),
                f.TYPE_FIXED64:  ('fixed64',  '0'),
                f.TYPE_FIXED32:  ('fixed32',  '0'),
                f.TYPE_BOOL:     ('bool',     'false'),
                f.TYPE_STRING:   ('string',   '""'),
                f.TYPE_BYTES:    ('bytes',    ''),
                f.TYPE_UINT32:   ('uint32',   '0'),
                f.TYPE_SFIXED32: ('sfixed32', '0'),
                f.TYPE_SFIXED64: ('sfixed64', '0'),
                f.TYPE_SINT32:   ('sint32',   '0'),
                f.TYPE_SINT64:   ('sint64',   '0')
            }[f.type]

            self.type = type_info[0]
            self.default_value = f.default_value if f.default_value else type_info[1]

        # If we are repeated or a pointer our default is changed
        if self.repeated:
            self.default_value = ''
        elif self.pointer:
            self.default_value = 'nullptr'

        # Since our cpp_type is used a lot, precalculate it
        self.cpp_type, self.special_cpp_type = self.get_cpp_type_info()

    def get_cpp_type_info(self):

        t = self.type

        # We are special unless we are not
        special = True

        vector_regex = re.compile(r'^\.message\.([fiuc]?)vec([2-4]?)$')
        matrix_regex = re.compile(r'^\.message\.([fiuc]?)mat([2-4]{0,2})$')

        # Check if it is a map field
        if self.map_type:
            t = '::std::map<{}, {}>'.format(t[0].cpp_type, t[1].cpp_type)

        # Check for matrix and vector types
        elif vector_regex.match(t):
            r = vector_regex.match(t)
            t = '::message::math::{}vec{}'.format(r.group(1), r.group(2))
        elif matrix_regex.match(t):
            r = matrix_regex.match(t)
            t = '::message::math::{}mat{}'.format(r.group(1), r.group(2))

        # Transform and rotation types map to the Transform classes
        elif t == '.message.Transform2D':
            t = '::message::math::Transform2D'
        elif t == '.message.Transform3D':
            t = '::message::math::Transform3D'
        elif t == '.message.Rotation2D':
            t = '::message::math::Rotation2D'
        elif t == '.message.Rotation3D':
            t = '::message::math::Rotation3D'

        # Timestamps and durations map to real time/duration classes
        elif t == '.google.protobuf.Timestamp':
            t = '::NUClear::clock::time_point'
        elif t == '.google.protobuf.Duration':
            t = '::NUClear::clock::duration'

        # Struct types map to YAML nodes
        elif t == '.google.protobuf.Struct':
            t = '::YAML::Node'

        # Standard types get mapped to their appropriate type
        elif t in ['double', 'float', 'bool']:
            # double and float and bool are fine as is
            special = False
        elif t in ['int64', 'sint64', 'sfixed64']:
            t = 'int64_t'
            special = False
        elif t in ['uint64', 'fixed64']:
            t = 'uint64_t'
            special = False
        elif t in ['int32', 'sint32', 'sfixed32']:
            t = 'int32_t'
            special = False
        elif t in ['uint32', 'fixed32']:
            t = 'uint32_t'
            special = False
        elif t in ['string']:
            t = '::std::string'
            special = False
        elif t in ['bytes']:
            t = '::std::vector<uint8_t>'
        # Otherwise we assume it's a normal type and let it work out its scoping
        else:
            t = '::'.join(t.split('.'))
            special = False

        # If we are using a pointer type do the manipulation here
        if self.pointer == PointerType['RAW']:
            t = '{}*'.format(t)
        elif self.pointer == PointerType['SHARED']:
            t = '::std::shared_ptr<{}>'.format(t)
        elif self.pointer == PointerType['UNIQUE']:
            t = '::std::unique_ptr<{}>'.format(t)

        # If it's a repeated field, and not a map, it's a vector
        if self.repeated and not self.map_type:
            t = '::std::vector<{}>'.format(t)

        return t, special

    def generate_cpp_header(self):
        return '{} {};'.format(self.cpp_type, to_camel_case(self.name))

    def __repr__(self):
        return "%r" % (self.__dict__)


class Message:
    def __init__(self, m, context):
        self.name = m.name
        self.fqn = '{}.{}'.format(context.fqn, self.name)
        self.enums = [Enum(e, self) for e in m.enum_type]

        # Get all the submessages that are not map entries
        self.submessages = [Message(n, self) for n in m.nested_type if not n.options.map_entry]
        self.oneofs = []

        for n in m.nested_type:
            if n.options.map_entry:
                Field.map_types['{}.{}'.format(self.fqn, n.name)] = (Field(n.field[0], self), Field(n.field[1], self))

        # All fields that are not a part of oneof
        self.fields = [Field(f, self) for f in m.field if f.oneof_index == 0]

        # m.name is the name of the message
        # m.field contains the fields
        # m.extension contains any exntesions (error don't use)
        # m.nested_type contains submessages
        # m.enum_type contains held enum types
        # m.extension_range contains extension ranges
        # m.oneof_decl contaions all one_of declarations
        # m.options contains options (message_set_wire_format no_standard_descriptor_accessor deprecated map_entry uninterpreted_option)
        # m.reserved_range and m.reserved_name contains names/field numbers not to use

    def generate_cpp_header(self):

        # Protobuf name
        protobuf_name = '::'.join(('.protobuf' + self.fqn).split('.'))

        # Make our value pairs
        fields = indent('\n'.join(['{}'.format(v.generate_cpp_header()) for v in self.fields]))

        # Get all our enums
        enums = indent('\n\n'.join([e.generate_cpp_header() for e in self.enums]))

        # Get all our submessages
        submessages = indent('\n\n'.join([m.generate_cpp_header() for m in self.submessages]))

        # Make our constructors
        constructors = []
        # If we don't have any fields, it is all very easy
        if not self.fields:
            # DEFAULT CONSTRUCTOR
            constructors.append('{}() {{}}'.format(self.name))
            # PROTOBUF CONSTRUCTOR
            constructors.append('{}(const {}&) {{}}'.format(self.name, protobuf_name))
        else:
            # CONSTRUCTOR WITH DEFAULT ARGUMENTS

            # TODO if no default, just make it a construction
            # TODO default timestamp is now

            field_list = ', '.join(['{} const& _{} = {}'.format(v.cpp_type, to_camel_case(v.name), v.default_value if v.default_value else '{}()'.format(v.cpp_type)) for v in self.fields])
            field_set = ', '.join(['{0}(_{0})'.format(to_camel_case(v.name)) for v in self.fields])
            constructors.append('{}({}) : {} {{}}'.format(self.name, field_list, field_set))

            # PROTOBUF CONSTRUCTOR
            protobuf_constructor = []
            protobuf_constructor.append('{}(const {}& proto) {{'.format(self.name, protobuf_name));
            for v in self.fields:

                if v.pointer:
                    print 'TODO HANDLE POINTER CASES'

                elif v.map_type:
                    if v.type[1].bytes_type:
                        protobuf_constructor.append(indent('for (auto& _v : proto.{}()) {{'.format(v.name)))
                        protobuf_constructor.append(indent('{0}[_v.first].insert(std::end({0}[_v.first]), std::begin(_v.second), std::end(_v.second));'.format(to_camel_case(v.name)), 8))
                        protobuf_constructor.append(indent('}'))

                    elif v.type[1].special_cpp_type:
                        protobuf_constructor.append(indent('for (auto& _v : proto.{}()) {{'.format(v.name)))
                        protobuf_constructor.append(indent('{}[_v.first] << _v.second;'.format(to_camel_case(v.name)), 8))
                        protobuf_constructor.append(indent('}'))

                    else: # Basic and other types are handled the same
                        protobuf_constructor.append(indent('{0}.insert(std::begin(proto.{1}()), std::end(proto.{1}()));'.format(to_camel_case(v.name), v.name), 8))

                elif v.repeated:
                    if v.bytes_type:
                        protobuf_constructor.append(indent('{}.resize(proto.{}_size());'.format(to_camel_case(v.name), v.name)))
                        protobuf_constructor.append(indent('for (size_t _i = 0; _i < {}.size(); ++_i) {{'.format(to_camel_case(v.name))))
                        protobuf_constructor.append(indent('{0}[_i].insert(std::end({0}[_i]), std::begin(proto.{1}(_i)), std::end(proto.{1}(_i)));'.format(to_camel_case(v.name), v.name), 8))
                        protobuf_constructor.append(indent('}'))

                    elif v.special_cpp_type:
                        # Add the top of our for loop for the repeated field
                        protobuf_constructor.append(indent('{}.resize(proto.{}_size());'.format(to_camel_case(v.name), v.name)))
                        protobuf_constructor.append(indent('for (size_t _i = 0; _i < {}.size(); ++_i) {{'.format(to_camel_case(v.name))))
                        protobuf_constructor.append(indent('{}[_i] << proto.{}(_i);'.format(to_camel_case(v.name), v.name), 8))
                        protobuf_constructor.append(indent('}'))

                    else: # Basic and other types are handled the same
                        protobuf_constructor.append(indent('{0}.insert(std::end({0}), std::begin(proto.{1}()), std::end(proto.{1}()));'.format(to_camel_case(v.name), v.name)))

                else:
                    if v.bytes_type:
                        protobuf_constructor.append(indent('{0}.insert(std::end({0}), std::begin(proto.{1}()), std::end(proto.{1}()));'.format(to_camel_case(v.name), v.name)))

                    elif v.special_cpp_type:
                        protobuf_constructor.append(indent('{} << proto.{}();'.format(to_camel_case(v.name), v.name)))

                    else: # Basic and other types are handled the same
                        protobuf_constructor.append(indent('{} = proto.{}();'.format(to_camel_case(v.name), v.name)))

            protobuf_constructor.append('}')
            constructors.append('\n'.join(protobuf_constructor))

        constructors = indent('\n\n'.join(constructors))

        # Make our converters
        converters = []
        # If we don't have any fields, it is all very easy
        if not self.fields:
            # PROTOBUF CONVERTER
            converters.append('inline operator {0}() const {{ return {0}(); }}'.format(protobuf_name))
        else:
            # PROTOBUF CONVERTER
            protobuf_converter = []
            protobuf_converter.append('inline operator {}() const {{'.format(protobuf_name))
            protobuf_converter.append(indent('{} proto;'.format(protobuf_name)))
            for v in self.fields:

                if v.pointer:
                    print 'TODO HANDLE POINTER CASES'

                elif v.map_type:
                    # Add the top of our for loop for the repeated field
                    protobuf_converter.append(indent('for (auto& _v : {}) {{'.format(to_camel_case(v.name))))

                    if v.type[1].bytes_type:
                        protobuf_converter.append(indent('(*proto.mutable_{}())[_v.first].append(std::begin(_v.second), std::end(_v.second));'.format(v.name), 8))
                    elif v.type[1].special_cpp_type:
                        protobuf_converter.append(indent('(*proto.mutable_{}())[_v.first] << _v.second;'.format(v.name), 8))
                    else: # Basic and others are handled the same
                        protobuf_converter.append(indent('(*proto.mutable_{}())[_v.first] = _v.second;'.format(v.name), 8))

                    protobuf_converter.append(indent('}'))

                elif v.repeated:
                    # Add the top of our for loop for the repeated field
                    protobuf_converter.append(indent('for (auto& _v : {}) {{'.format(to_camel_case(v.name))))

                    if v.bytes_type:
                        protobuf_converter.append(indent('proto.add_{}()->append(std::begin(_v), std::end(_v));'.format(v.name), 8))
                    elif v.special_cpp_type:
                        protobuf_converter.append(indent('*proto.add_{}() << _v;'.format(v.name), 8))
                    elif v.basic:
                        protobuf_converter.append(indent('proto.add_{}(_v);'.format(v.name), 8))
                    else:
                        protobuf_converter.append(indent('*proto.add_{}() = _v;'.format(v.name), 8))

                    protobuf_converter.append(indent('}'))

                else:
                    if v.bytes_type:
                        protobuf_converter.append(indent('proto.mutable_{0}()->append(std::begin({1}), std::end({1}));'.format(v.name, to_camel_case(v.name)), 8))
                    elif v.special_cpp_type:
                        protobuf_converter.append(indent('*proto.mutable_{}() << {};'.format(v.name, to_camel_case(v.name))))
                    elif v.basic:
                        protobuf_converter.append(indent('proto.set_{}({});'.format(v.name, to_camel_case(v.name))))
                    else:
                        protobuf_converter.append(indent('*proto.mutable_{}() = {};'.format(v.name, to_camel_case(v.name))))

            protobuf_converter.append(indent('return proto;'))
            protobuf_converter.append('}')
            converters.append('\n'.join(protobuf_converter))

        converters = indent('\n\n'.join(converters))

        template = textwrap.dedent("""\
            struct {name} {{
                // Enum Definitions
            {enums}
                // Submessage Definitions
            {submessages}
                // Constructors
            {constructors}
                // Converters
            {converters}
                // Fields
            {fields}
            }};
            """);

        # TODO
        # {name}() : defaultvalues {{}}
        # {name}(const YAML::Node& node) {{ CONVERT }}
        # {name}(const PyObject* pyobj) {{ CONVERT }}

        # operator YAML::Node() {{ CONVERT TO YAML NODE }}
        # operator PyObject*() {{ WRAP IN A PYOBJECT }}

        return template.format(
            name=self.name,
            enums=enums,
            submessages=submessages,
            constructors=constructors,
            converters=converters,
            fields=fields
        )

    def __repr__(self):
        return "%r" % (self.__dict__)


class Enum:
    def __init__(self, e, context):
        self.name = e.name
        self.fqn = '{}.{}'.format(context.fqn, self.name)
        self.values = [(v.name, v.number) for v in e.value]
        # e.name contains the name of the enum
        # e.value is a list of enum values
        # e.options is a set of enum options (allow_alias, deprecated, list of uninterpreted options)
        # e.value[].name is the name of the constant
        # e.value[].number is the number assigned
        # e.value[].options is a set of enum options (deprecated, list of uninterpreted_option)

    def generate_cpp_header(self):

        # Make our value pairs
        values = indent('\n'.join(['{} = {}'.format(v[0], v[1]) for v in self.values]), 8)
        values = ',\n'.join([v for v in values.splitlines()])

        # Make our switch statement pairs
        switches = indent('\n'.join(['case Value::{}: return "{}";'.format(v[0], v[0]) for v in self.values]), 12)

        # Make our if chain
        if_chain = indent('\n'.join(['if (str == "{}") value = Value::{};'.format(v[0], v[0]) for v in self.values]), 8)

        # Get our default value
        default_value = dict([reversed(v) for v in self.values])[0]

        # Make our fancy enums
        template = textwrap.dedent("""\
            struct {name} {{
                enum Value {{
            {values}
                }};
                Value value;

                {name}() : value(Value::{default_value}) {{}}

                {name}(const int& v) : value(static_cast<Value>(v)) {{}}

                {name}(const Value& value)
                : value(value) {{}}

                {name}(const std::string& str) {{
            {if_chain}
                    throw std::runtime_error("String did not match any enum for {name}");
                }}

                {name}(const {protobuf_name}& p) {{
                    value = static_cast<Value>(p);
                }}

                bool operator <(const {name}& other) const {{
                    return value < other.value;
                }}

                bool operator <(const {name}::Value& other) const {{
                    return value < other;
                }}

                bool operator ==(const {name}& other) const {{
                    return value == other.value;
                }}

                bool operator ==(const {name}::Value& other) const {{
                    return value == other;
                }}

                inline operator Value() const {{
                    return value;
                }}

                inline operator std::string() const {{
                    switch(value) {{
            {switches}
                        default:
                            throw std::runtime_error("enum {name}'s value is corrupt, unknown value stored");
                    }}
                }}

                inline operator {protobuf_name}() const {{
                    return static_cast<{protobuf_name}>(value);
                }}
            }};
            """)

        return template.format(
            name=self.name,
            protobuf_name='::'.join(('.protobuf' + self.fqn).split('.')),
            values=values,
            default_value=default_value,
            if_chain=if_chain,
            switches=switches
        )

    def __repr__(self):
        return "%r" % (self.__dict__)


class File:
    def __init__(self, f):
        self.package = f.package
        self.name = f.name
        self.fqn = '.{}'.format(self.package)
        self.dependencies = [d for d in f.dependency]
        self.enums = [Enum(e, self) for e in f.enum_type]
        self.messages = [Message(m, self) for m in f.message_type]

        # f.fileoptions contains many things including unrecognised options (java_package java_outer_classname java_multiple_files java_generate_equals_and_hash java_string_check_utf8 optimize_for go_package cc_generic_services java_generic_services py_generic_services deprecated cc_enable_arenas objc_class_prefix csharp_namespace javanano_use_deprecated_package uninterpreted_option)
        # f.syntax tells if it is proto2 or proto3

    def generate_cpp_header(self):

        define = '{}_H'.format('_'.join([s.upper() for s in self.name[:-6].strip().split('/')]))
        parts = self.package.split('.')
        ns_open = '\n'.join(['namespace {} {{'.format(x) for x in parts])
        ns_close = '\n'.join('}' * len(parts))
        enums = indent('\n\n'.join([e.generate_cpp_header() for e in self.enums]))
        messages = indent('\n\n'.join([m.generate_cpp_header() for m in self.messages]))

        # By default include some useful headers
        includes = {
            '1<cstdint>',
            '2<string>',
            '2<map>',
            '2<vector>',
            '2<memory>',
            '4"{}"'.format(self.name[:-6] + '.pb.h')
        }

        # We use a dirty hack here of putting a priority on each header
        # to make the includes be in a better order
        for d in self.dependencies:
            if d in ['message/Vector.proto', 'message/Matrix.proto']:
                includes.add('4"utility/conversion/matrix_types.h"')
                includes.add('4"utility/conversion/proto_matrix.h"')
            if d in ['message/Transform.proto']:
                includes.add('4"utility/math/matrix/Transform2D.h"')
                includes.add('4"utility/math/matrix/Transform3D.h"')
                includes.add('4"utility/conversion/proto_transform.h"')
            if d in ['message/Rotation.proto']:
                includes.add('4"utility/math/matrix/Rotation2D.h"')
                includes.add('4"utility/math/matrix/Rotation3D.h"')
                includes.add('4"utility/conversion/proto_rotation.h"')
            elif d in ['google/protobuf/timestamp.proto', 'google/protobuf/duration.proto']:
                includes.add('3<nuclear_bits/clock.hpp>')
                includes.add('4"utility/conversion/proto_time.h"')
            elif d in ['google/protobuf/struct.proto']:
                includes.add('3<yaml-cpp/yaml.h>')
                includes.add('4"utility/conversion/proto_yaml.h"')
            else:
                includes.add('4"{}"'.format(d[:-6] + '.h'))
        # Don't forget to remove the first character
        includes = '\n'.join(['#include {}'.format(i[1:]) for i in sorted(list(includes))])

        template = textwrap.dedent("""\
            #ifndef {define}
            #define {define}

            {includes}

            {openNamespace}

                // Enum Definitions
            {enums}
                // Message Definitions
            {messages}

            {closeNamespace}

            #endif  // {define}
            """)

        return template.format(
            define=define,
            includes=includes,
            openNamespace=ns_open,
            enums=enums,
            messages=messages,
            closeNamespace=ns_close
        )

    def generate_cpp_impl(self):
        return '#include "{}"'.format(self.name[:-6] + '.h')

    def __repr__(self):
        return "%r" % (self.__dict__)


base_file = sys.argv[1]

with open('{}.pb'.format(base_file), 'r') as f:
    # Load the descriptor protobuf file
    d = FileDescriptorSet()
    d.ParseFromString(f.read())

    # Check that there is only one file
    assert(len(d.file) == 1)

    b = File(d.file[0])

    with open('{}.cpp'.format(base_file), 'w') as f:
        f.write(b.generate_cpp_impl())

    with open('{}.h'.format(base_file), 'w') as f:
        f.write(b.generate_cpp_header())
