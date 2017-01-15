#!/usr/bin/env python3

from generator.textutil import indent, dedent
from generator.Field import Field
from generator.Enum import Enum


class Message:
    def __init__(self, m, context):
        self.name = m.name
        self.fqn = '{}.{}'.format(context.fqn, self.name)
        self.include_path = context.include_path
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

    def generate_default_constructor(self):

        # Fully qualified c++ name
        cpp_fqn = '::'.join(self.fqn.split('.'));

        # If we are empty it's easy
        if not self.fields:
            return ('{}();'.format(self.name),
                    '{}::{}() {{}}'.format(cpp_fqn, self.name))
        else:
            field_list = ', '.join(['{} const& _{}'.format(v.cpp_type, v.name) for v in self.fields])
            default_field_list = ', '.join(['{} const& _{} = {}'.format(v.cpp_type, v.name, v.default_value if v.default_value else '{}()'.format(v.cpp_type)) for v in self.fields])
            field_set = ', '.join(['{0}(_{0})'.format(v.name) for v in self.fields])

            return ('{}({});'.format(self.name, default_field_list),
                    '{}::{}({}) : {} {{}}'.format(cpp_fqn, self.name, field_list, field_set))

    def generate_protobuf_constructor(self):

        # Fully qualified c++ name
        cpp_fqn = '::'.join(self.fqn.split('.'))

        # Protobuf name
        protobuf_name = '::'.join(('.protobuf' + self.fqn).split('.'))

        # If we are empty it's easy
        if not self.fields:
            return ('{}(const {}&);'.format(self.name, protobuf_name),
                    '{}::{}(const {}&) {{}}'.format(cpp_fqn, self.name, protobuf_name))
        else:
            lines = ['{}::{}(const {}& proto) {{'.format(cpp_fqn, self.name, protobuf_name)]

            for v in self.fields:
                if v.pointer:
                    print('TODO HANDLE POINTER CASES')

                elif v.map_type:
                    if v.type[1].bytes_type:
                        lines.append(indent('for (auto& _v : proto.{}()) {{'.format(v.name)))
                        lines.append(indent('{0}[_v.first].insert(std::end({0}[_v.first]), std::begin(_v.second), std::end(_v.second));'.format(v.name), 8))
                        lines.append(indent('}'))

                    elif v.type[1].special_cpp_type:
                        lines.append(indent('for (auto& _v : proto.{}()) {{'.format(v.name.lower())))
                        lines.append(indent('message::conversion::convert({}[_v.first], _v.second);'.format(v.name), 8))
                        lines.append(indent('}'))

                    else:  # Basic and other types are handled the same
                        lines.append(indent('{0}.insert(std::begin(proto.{1}()), std::end(proto.{1}()));'.format(v.name, v.name.lower()), 8))

                elif v.repeated:
                    if v.bytes_type:
                        lines.append(indent('{0}.resize(proto.{0}_size());'.format(v.name.lower())))
                        lines.append(indent('for (size_t _i = 0; _i < {0}.size(); ++_i) {{'.format(v.name)))
                        lines.append(indent('{0}[_i].insert(std::end({0}[_i]), std::begin(proto.{1}(_i)), std::end(proto.{1}(_i)));'.format(v.name, v.name.lower()), 8))
                        lines.append(indent('}'))

                    elif v.special_cpp_type:
                        if v.array_size > 0:
                            lines.append(indent('for (size_t _i = 0; _i < {0}.size() && _i < size_t(proto.{1}_size()); ++_i) {{'.format(v.name, v.name.lower())))
                            lines.append(indent('message::conversion::convert({0}[_i], proto.{1}(_i));'.format(v.name, v.name.lower()), 8))
                            lines.append(indent('}'))
                        else:
                            # Add the top of our for loop for the repeated field
                            lines.append(indent('{0}.resize(proto.{1}_size());'.format(v.name, v.name.lower())))
                            lines.append(indent('for (size_t _i = 0; _i < {0}.size(); ++_i) {{'.format(v.name)))
                            lines.append(indent('message::conversion::convert({0}[_i], proto.{1}(_i));'.format(v.name, v.name.lower()), 8))
                            lines.append(indent('}'))

                    else:  # Basic and other types are handled the same
                        if v.array_size > 0:
                            lines.append(indent('for (size_t _i = 0; _i < {0}.size() && _i < size_t(proto.{1}_size()); ++_i) {{'.format(v.name, v.name.lower())))
                            lines.append(indent('{0}[_i] = proto.{1}(_i);'.format(v.name, v.name.lower()), 8))
                            lines.append(indent('}'))
                        else:
                            lines.append(indent('{0}.insert(std::end({0}), std::begin(proto.{1}()), std::end(proto.{1}()));'.format(v.name, v.name.lower())))

                else:
                    if v.bytes_type:
                        lines.append(indent('{0}.insert(std::end({0}), std::begin(proto.{1}()), std::end(proto.{1}()));'.format(v.name, v.name.lower())))

                    elif v.special_cpp_type:
                        lines.append(indent('message::conversion::convert({}, proto.{}());'.format(v.name, v.name.lower())))

                    else:  # Basic and other types are handled the same
                        lines.append(indent('{} = proto.{}();'.format(v.name, v.name.lower())))

            lines.append('}')

            return '{}(const {}& proto);'.format(self.name, protobuf_name), '\n'.join(lines)

    def generate_protobuf_converter(self):

        # Fully qualified c++ name
        cpp_fqn = '::'.join(self.fqn.split('.'))

        # Protobuf name
        protobuf_name = '::'.join(('.protobuf' + self.fqn).split('.'))

        # If we are empty it's easy
        if not self.fields:
            return ('operator {0}() const;'.format(protobuf_name),
                    '{0}::operator {1}() const {{\n    return {1}();\n}}'.format(cpp_fqn, protobuf_name))
        else:
            lines = ['{}::operator {}() const {{'.format(cpp_fqn, protobuf_name),
                     indent('{} proto;'.format(protobuf_name))]

            for v in self.fields:

                if v.pointer:
                    print('TODO HANDLE POINTER CASES')

                elif v.map_type:
                    # Add the top of our for loop for the repeated field
                    lines.append(indent('for (auto& _v : {}) {{'.format(v.name)))

                    if v.type[1].bytes_type:
                        lines.append(indent('(*proto.mutable_{}())[_v.first].append(std::begin(_v.second), std::end(_v.second));'.format(v.name), 8))
                    elif v.type[1].special_cpp_type:
                        lines.append(indent('message::conversion::convert((*proto.mutable_{}())[_v.first], _v.second);'.format(v.name), 8))
                    else: # Basic and others are handled the same
                        lines.append(indent('(*proto.mutable_{}())[_v.first] = _v.second;'.format(v.name), 8))

                    lines.append(indent('}'))

                elif v.repeated: # We don't need to handle array here specially because it's the same
                    # Add the top of our for loop for the repeated field
                    lines.append(indent('for (auto& _v : {}) {{'.format(v.name)))

                    if v.bytes_type:
                        lines.append(indent('proto.add_{}()->append(std::begin(_v), std::end(_v));'.format(v.name.lower()), 8))
                    elif v.special_cpp_type:
                        lines.append(indent('message::conversion::convert(*proto.add_{}(), _v);'.format(v.name.lower()), 8))
                    elif v.basic:
                        lines.append(indent('proto.add_{}(_v);'.format(v.name.lower()), 8))
                    else:
                        lines.append(indent('*proto.add_{}() = _v;'.format(v.name.lower()), 8))

                    lines.append(indent('}'))

                else:
                    if v.bytes_type:
                        lines.append(indent('proto.mutable_{0}()->append(std::begin({1}), std::end({1}));'.format(v.name.lower(), v.name), 8))
                    elif v.special_cpp_type:
                        lines.append(indent('message::conversion::convert(*proto.mutable_{}(), {});'.format(v.name.lower(), v.name)))
                    elif v.basic:
                        lines.append(indent('proto.set_{}({});'.format(v.name.lower(), v.name)))
                    else:
                        lines.append(indent('*proto.mutable_{}() = {};'.format(v.name.lower(), v.name)))

            lines.append(indent('return proto;'))
            lines.append('}')

            return 'operator {0}() const;'.format(protobuf_name), '\n'.join(lines)

    def generate_cpp(self):

        # Make our value pairs
        fields = indent('\n'.join(['{}'.format(v.generate_cpp_header()) for v in self.fields]))

        # Generate our protobuf class name
        protobuf_type = '::'.join(('.protobuf' + self.fqn).split('.'))

        # Generate our enums c++
        enums = [e.generate_cpp() for e in self.enums]
        enum_headers = indent('\n\n'.join([e[0] for e in enums]))
        enum_impls = ('\n\n'.join([e[1] for e in enums]))
        enum_python = ('\n\n'.join([e[2] for e in enums]))

        # Generate our submessage c++
        submessages = [s.generate_cpp() for s in self.submessages]
        submessage_headers = indent('\n\n'.join([s[0] for s in submessages]))
        submessage_impls = ('\n\n'.join([s[1] for s in submessages]))
        submessage_python = ('\n\n'.join([s[2] for s in submessages]))

        # Get our function code
        default_constructor = self.generate_default_constructor()
        protobuf_constructor = self.generate_protobuf_constructor()
        protobuf_converter = self.generate_protobuf_converter()

        constructor_headers = indent('\n\n'.join([default_constructor[0], protobuf_constructor[0]]))
        constructor_impl = '\n\n'.join([default_constructor[1], protobuf_constructor[1]])
        converter_headers = indent('\n\n'.join([protobuf_converter[0]]))
        converter_impl = '\n\n'.join([protobuf_converter[1]])

        header_template = dedent("""\
            struct alignas(16) {name} : public ::message::MessageBase<{name}> {{
                // Protobuf type
                using protobuf_type = {protobuf_type};

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
            }};""")

        impl_template = dedent("""\
            // Constructors
            {constructors}

            // Converters
            {converters}

            // Subenums
            {enums}

            // Submessages
            {submessages}""")

        python_template = dedent("""\
            // Local scope for this message
            {{
                // Use our context and assign a new one to a shadow
                auto shadow = pybind11::class_<{fqn}, std::shared_ptr<{fqn}>>(context, "{name}");

                // Shadow our context with our new context and declare our subclasses
                auto& context = shadow;
            {submessages}
            {enums}

                // Declare the functions on our class (which may use the ones in the subclasses)
                context
            {constructor}
            {message_members};
                context.def_static("include_path", [] {{
                    return "{include_path}";
                }});
            }}""")

        python_members = '\n'.join('.def_readwrite("{field}", &{fqn}::{field})'.format(field=f.name, fqn=self.fqn.replace('.', '::')) for f in self.fields)
        python_constructor = dedent("""\
            .def("__init__", [] ({name}& self, {args}) {{
                new (&self) {name}({vars});
            }}, {default_args})""").format(
            name=self.fqn.replace('.', '::'),
            args=', '.join('{} const& {}'.format(t.cpp_type, t.name) for t in self.fields),
            vars=', '.join(t.name for t in self.fields),
            default_args=', '.join('pybind11::arg("{}") = {}'.format(t.name, t.default_value if t.default_value else '{}()'.format(t.cpp_type)) for t in self.fields)
        )

        return header_template.format(
            name=self.name,
            enums=enum_headers,
            submessages=submessage_headers,
            constructors=constructor_headers,
            protobuf_type=protobuf_type,
            converters=converter_headers,
            fields=fields
        ), impl_template.format(
            constructors=constructor_impl,
            converters=converter_impl,
            enums=enum_impls,
            submessages=submessage_impls
        ), python_template.format(
            constructor=indent(python_constructor, 8),
            message_members=indent(python_members, 8),
            include_path=self.include_path,
            fqn=self.fqn.replace('.', '::'),
            name=self.name,
            submessages=indent(submessage_python),
            enums=indent(enum_python)
        )
