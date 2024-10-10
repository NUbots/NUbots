#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2016 NUbots
#
# This file is part of the NUbots codebase.
# See https://github.com/NUbots/NUbots for further info.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

from generator.Enum import Enum
from generator.Field import Field, PointerType
from generator.OneOfField import OneOfField
from generator.OneOfType import OneOfType
from generator.textutil import dedent, indent


class Message:
    def __init__(self, m, context):
        self.package = context.package
        self.name = m.name
        self.fqn = "{}.{}".format(context.fqn, self.name)
        self.include_path = context.include_path
        self.enums = [Enum(e, self) for e in m.enum_type]

        # Get all the submessages that are not map entries
        self.submessages = [Message(n, self) for n in m.nested_type if not n.options.map_entry]

        for n in m.nested_type:
            if n.options.map_entry:
                Field.map_types["{}.{}".format(self.fqn, n.name)] = (Field(n.field[0], self), Field(n.field[1], self))

        self.fields = []
        FIELD_ONEOF_INDEX = "oneof_index"
        for f in m.field:
            # Field is not a oneof
            if not f.HasField(FIELD_ONEOF_INDEX):
                self.fields.append(Field(f, self))
            else:
                # Field is part of a oneof. Find all fields that are part of the same oneof
                oneof_fields = [v for v in m.field if v.HasField(FIELD_ONEOF_INDEX) and v.oneof_index == f.oneof_index]

                # Create and add the oneof but only once when the first field is encountered for this oneof
                if oneof_fields.index(f) == 0:
                    oneof_name = m.oneof_decl[f.oneof_index].name
                    self.fields.append(OneOfField(oneof_name, oneof_fields, self))
                    self.submessages.append(OneOfType(oneof_name, oneof_fields, self))

    def generate_default_constructor(self):

        # Fully qualified c++ name
        cpp_fqn = "::".join(self.fqn.split("."))

        # If we are empty it's easy
        if not self.fields:
            return ("{}();".format(self.name), "{}::{}() = default;".format(cpp_fqn, self.name))
        else:
            # Basic types are const reference and others are copy/moved
            field_list = []
            default_field_list = []
            field_set = []
            for v in self.fields:
                # Work out if we should be copying a const reference or moving a trivial type
                if v.trivially_copyable:
                    field_list.append("{} const& {}".format(v.cpp_type, v.name))
                    field_set.append("{0}({0})".format(v.name))
                else:
                    field_list.append("{} {}".format(v.cpp_type, v.name))
                    field_set.append("{0}(std::move({0}))".format(v.name))
                default_field_list.append(
                    "{} = {}".format(field_list[-1], v.default_value if v.default_value else "{}()".format(v.cpp_type))
                )

            field_list = ", ".join(field_list)
            default_field_list = ", ".join(default_field_list)
            field_set = ", ".join(field_set)

            return (
                "{}({});".format(self.name, default_field_list),
                "{}::{}({}) : {} {{}}".format(cpp_fqn, self.name, field_list, field_set),
            )

    def generate_equality_operator(self):

        # Fully qualified c++ name
        cpp_fqn = "::".join(self.fqn.split("."))

        # If we are empty it's easy
        if not self.fields:
            return (
                "bool operator== (const {}& /*other*/) const;".format(self.name),
                "bool {}::operator== (const {}& /*other*/) const {{ return true; }}".format(cpp_fqn, self.name),
            )
        else:
            equality_test = " && ".join(["{0} == other.{0}".format(v.name) for v in self.fields])

            return (
                "bool operator== (const {}& other) const;".format(self.name),
                "bool {}::operator== (const {}& other) const {{ return {}; }}".format(
                    cpp_fqn, self.name, equality_test
                ),
            )

    def generate_rule_of_five(self):

        raw_pointer = [v.name for v in self.fields if v.pointer and v.pointer == PointerType["RAW"]]
        raw_pointer_warning = '#pragma message ( "WARNING: The following fields in {0} are raw pointers and copying or moving will copy the raw pointer address: {1}") \n'.format(
            self.name, ", ".join(raw_pointer)
        )

        rule_of_five = dedent(
            """\
            {warning}
            {name}(const {name}&) = default;
            {name}({name}&&) = default;
            ~{name}() = default;
            {name}& operator=(const {name}&) = default;
            {name}& operator=({name}&&) = default;"""
        )

        return (rule_of_five.format(name=self.name, warning=raw_pointer_warning if raw_pointer else ""), "")

    def generate_protobuf_constructor(self):

        # Fully qualified c++ name
        cpp_fqn = "::".join(self.fqn.split("."))

        # Protobuf name
        protobuf_name = "::".join((".protobuf" + self.fqn).split("."))

        # If we are empty it's easy
        if not self.fields:
            return (
                "{}(const {}& /*proto*/);".format(self.name, protobuf_name),
                "{}::{}(const {}& /*proto*/) {{}}".format(cpp_fqn, self.name, protobuf_name),
            )
        else:
            lines = ["{cpp_fqn}::{cpp_name}(const {proto_name}& proto) {member_init} {{"]

            member_inits = []
            for v in self.fields:
                if v.pointer:
                    member_inits.append("{}(nullptr)".format(v.name))
                    print("TODO HANDLE POINTER CASES")

                elif v.map_type:
                    if v.type[1].bytes_type:
                        lines.append(indent("for (const auto& v : proto.{}()) {{".format(v.name)))
                        lines.append(
                            indent(
                                "{0}[v.first].insert(std::end({0}[v.first]), std::begin(v.second), std::end(v.second));".format(
                                    v.name
                                ),
                                8,
                            )
                        )
                        lines.append(indent("}"))

                    elif v.type[1].special_cpp_type:
                        lines.append(indent("for (const auto& v : proto.{}()) {{".format(v.name.lower())))
                        lines.append(
                            indent(
                                "{0}[v.first] = message::conversion::convert<{1}>(v.second);".format(
                                    v.name, v.cpp_type
                                ),
                                8,
                            )
                        )
                        lines.append(indent("}"))

                    else:  # Basic and other types are handled the same
                        lines.append(
                            indent(
                                "{0}.insert(std::begin(proto.{1}()), std::end(proto.{1}()));".format(
                                    v.name, v.name.lower()
                                ),
                                8,
                            )
                        )

                elif v.repeated:
                    if v.bytes_type:
                        lines.append(indent("{0}.resize(proto.{0}_size());".format(v.name.lower())))
                        lines.append(indent("for (int i = 0; i < int({0}.size()); ++i) {{".format(v.name)))
                        lines.append(
                            indent(
                                "{0}[i].insert(std::end({0}[i]), std::begin(proto.{1}(i)), std::end(proto.{1}(i)));".format(
                                    v.name, v.name.lower()
                                ),
                                8,
                            )
                        )
                        lines.append(indent("}"))

                    elif v.special_cpp_type:
                        if v.array_size > 0:
                            lines.append(
                                indent(
                                    "for (size_t i = 0; i < {0}.size() && i < size_t(proto.{1}_size()); ++i) {{".format(
                                        v.name, v.name.lower()
                                    )
                                )
                            )
                            lines.append(
                                indent(
                                    "{0}[i] = message::conversion::convert<{1}::value_type>(proto.{2}(i));".format(
                                        v.name, v.cpp_type, v.name.lower()
                                    ),
                                    8,
                                )
                            )
                            lines.append(indent("}"))
                        else:
                            # Add the top of our for loop for the repeated field
                            lines.append(indent("{0}.resize(proto.{1}_size());".format(v.name, v.name.lower())))
                            lines.append(indent("for (int i = 0; i < int({0}.size()); ++i) {{".format(v.name)))
                            lines.append(
                                indent(
                                    "{0}[i] = message::conversion::convert<{1}::value_type>(proto.{2}(i));".format(
                                        v.name, v.cpp_type, v.name.lower()
                                    ),
                                    8,
                                )
                            )
                            lines.append(indent("}"))

                    else:  # Basic and other types are handled the same
                        if v.array_size > 0:
                            lines.append(
                                indent(
                                    "for (size_t i = 0; i < {0}.size() && i < size_t(proto.{1}_size()); ++i) {{".format(
                                        v.name, v.name.lower()
                                    )
                                )
                            )
                            lines.append(indent("{0}[i] = proto.{1}(i);".format(v.name, v.name.lower()), 8))
                            lines.append(indent("}"))
                        else:
                            member_inits.append(
                                "{0}(std::begin(proto.{1}()), std::end(proto.{1}()))".format(v.name, v.name.lower())
                            )

                elif v.one_of:
                    lines.append(indent("switch (proto.{}_case()) {{").format(v.name.lower()))
                    for oneof_field in v.oneof_fields:
                        lines.append(indent("case {}: {{".format(oneof_field.number), 8))

                        if oneof_field.bytes_type:
                            lines.append(
                                indent(
                                    dedent(
                                        """\
                                        {0}.{1} = {2};
                                        {0}.{1}.insert(std::end({0}.{1}), std::begin(proto.{3}()), std::end(proto.{3}()));""".format(
                                            v.name,
                                            oneof_field.name,
                                            oneof_field.default_value,
                                            oneof_field.name.lower(),
                                        )
                                    ),
                                    12,
                                )
                            )

                        elif oneof_field.special_cpp_type:
                            lines.append(
                                indent(
                                    dedent(
                                        """\
                                        {0}.{1} = {2};
                                        {0}.{1} = message::conversion::convert<{4}>(proto.{3}());""".format(
                                            v.name,
                                            oneof_field.name,
                                            oneof_field.default_value,
                                            oneof_field.name.lower(),
                                            v.cpp_name,
                                        )
                                    ),
                                    12,
                                )
                            )

                        else:  # Basic and other types are handled the same
                            lines.append(
                                indent(
                                    "{0}.{1} = proto.{2}();".format(v.name, oneof_field.name, oneof_field.name.lower()),
                                    12,
                                )
                            )

                        lines.append(indent("} break;", 8))
                    lines.append(indent("default: {}.reset(); break;".format(v.name), 8))
                    lines.append(indent("}"))

                else:
                    if v.bytes_type:
                        member_inits.append(
                            "{0}(std::begin(proto.{1}()), std::end(proto.{1}()))".format(v.name, v.name.lower())
                        )

                    elif v.special_cpp_type:
                        member_inits.append(
                            "{0}(message::conversion::convert<{1}>(proto.{2}()))".format(
                                v.name, v.cpp_type, v.name.lower()
                            )
                        )

                    else:  # Basic and other types are handled the same
                        member_inits.append("{}(proto.{}())".format(v.name, v.name.lower()))

            lines.append("}")

            lines[0] = lines[0].format(
                cpp_fqn=cpp_fqn,
                cpp_name=self.name,
                proto_name=protobuf_name,
                member_init=": {}".format(", ".join(member_inits)) if len(member_inits) > 0 else "",
            )

            return (
                "{}(const {}& proto);".format(self.name, protobuf_name),
                "\n".join(lines),
            )

    def generate_protobuf_converter(self):

        # Fully qualified c++ name
        cpp_fqn = "::".join(self.fqn.split("."))

        # Protobuf name
        protobuf_name = "::".join((".protobuf" + self.fqn).split("."))

        # If we are empty it's easy
        if not self.fields:
            return (
                "operator {0}() const;".format(protobuf_name),
                "{0}::operator {1}() const {{\n    return {{}};\n}}".format(cpp_fqn, protobuf_name),
            )
        else:
            lines = [
                "{}::operator {}() const {{".format(cpp_fqn, protobuf_name),
                indent("{} proto;".format(protobuf_name)),
            ]

            for v in self.fields:

                if v.pointer:
                    print("TODO HANDLE POINTER CASES")

                elif v.map_type:
                    # Add the top of our for loop for the repeated field
                    lines.append(indent("for (const auto& v : {}) {{".format(v.name)))

                    if v.type[1].bytes_type:
                        lines.append(
                            indent(
                                "(*proto.mutable_{}())[v.first].append(std::begin(v.second), std::end(v.second));".format(
                                    v.name.lower()
                                ),
                                8,
                            )
                        )
                    elif v.type[1].special_cpp_type:
                        lines.append(
                            indent(
                                "(*proto.mutable_{0}())[v.first] = message::conversion::convert<{1}>(v.second);".format(
                                    v.name.lower(), v.cpp_type
                                ),
                                8,
                            )
                        )
                    else:  # Basic and others are handled the same
                        lines.append(indent("(*proto.mutable_{}())[v.first] = v.second;".format(v.name.lower()), 8))

                    lines.append(indent("}"))

                elif v.repeated:  # We don't need to handle array here specially because it's the same
                    # Add the top of our for loop for the repeated field
                    lines.append(indent("for (const auto& v : {}) {{".format(v.name)))

                    if v.bytes_type:
                        lines.append(
                            indent("proto.add_{}()->append(std::begin(v), std::end(v));".format(v.name.lower()), 8)
                        )
                    elif v.special_cpp_type:
                        lines.append(
                            indent(
                                "*proto.add_{0}() = message::conversion::convert<{1}::value_type>(v);".format(
                                    v.name.lower(), v.cpp_type
                                ),
                                8,
                            )
                        )
                    elif v.basic:
                        lines.append(indent("proto.add_{}(v);".format(v.name.lower()), 8))
                    else:
                        lines.append(indent("*proto.add_{}() = v;".format(v.name.lower()), 8))

                    lines.append(indent("}"))

                elif v.one_of:
                    lines.append(indent("switch ({}.val_index) {{").format(v.name))
                    for oneof_field in v.oneof_fields:
                        lines.append(indent("case {}: {{".format(oneof_field.number), 8))

                        if oneof_field.bytes_type:
                            lines.append(
                                indent(
                                    "proto.mutable_{0}()->append(std::begin({1}.{2}.get()), std::end({1}.{2}.get()));".format(
                                        oneof_field.name.lower(), v.name, oneof_field.name
                                    ),
                                    12,
                                )
                            )
                        elif oneof_field.special_cpp_type:
                            lines.append(
                                indent(
                                    "*proto.mutable_{0}() = message::conversion::convert<{1}>({2}.{3}.get());".format(
                                        oneof_field.name.lower(), v.cpp_type, v.name, oneof_field.name
                                    ),
                                    12,
                                )
                            )
                        elif oneof_field.basic:
                            lines.append(
                                indent(
                                    "proto.set_{0}({1}.{2}.get());".format(
                                        oneof_field.name.lower(), v.name, oneof_field.name
                                    ),
                                    12,
                                )
                            )
                        else:
                            lines.append(
                                indent(
                                    "*proto.mutable_{0}() = {1}.{2}.get();".format(
                                        oneof_field.name.lower(), v.name, oneof_field.name
                                    ),
                                    12,
                                )
                            )

                        lines.append(indent("} break;", 8))
                    lines.append(indent("default: break;", 8))
                    lines.append(indent("}"))

                else:
                    if v.bytes_type:
                        lines.append(
                            indent(
                                "proto.mutable_{0}()->append(std::begin({1}), std::end({1}));".format(
                                    v.name.lower(), v.name
                                ),
                                8,
                            )
                        )
                    elif v.special_cpp_type:
                        lines.append(
                            indent(
                                "*proto.mutable_{0}() = message::conversion::convert<{1}>({2});".format(
                                    v.name.lower(), v.cpp_type, v.name
                                )
                            )
                        )
                    elif v.basic:
                        lines.append(indent("proto.set_{}({});".format(v.name.lower(), v.name)))
                    else:
                        lines.append(indent("*proto.mutable_{}() = {};".format(v.name.lower(), v.name)))

            lines.append(indent("return proto;"))
            lines.append("}")

            return "operator {0}() const;".format(protobuf_name), "\n".join(lines)

    def generate_cpp(self):

        # Make our value pairs
        fields = indent("\n".join(["{}".format(v.generate_cpp_header()) for v in self.fields]))

        # Generate our protobuf class name
        protobuf_type = "::".join((".protobuf" + self.fqn).split("."))

        # Generate our enums c++
        enums = [e.generate_cpp() for e in self.enums]
        enum_headers = indent("\n\n".join([e[0] for e in enums]))
        enum_impls = "\n\n".join([e[1] for e in enums])
        enum_python = "\n\n".join([e[2] for e in enums])

        # Generate our submessage c++
        submessages = [s.generate_cpp() for s in self.submessages]
        submessage_headers = indent("\n\n".join([s[0] for s in submessages]))
        submessage_impls = "\n\n".join([s[1] for s in submessages])
        submessage_python = "\n\n".join([s[2] for s in submessages])

        # Get our function code
        default_constructor = self.generate_default_constructor()
        rule_of_five = self.generate_rule_of_five()
        protobuf_constructor = self.generate_protobuf_constructor()
        protobuf_converter = self.generate_protobuf_converter()
        equality_operator = self.generate_equality_operator()

        constructor_headers = indent(
            "\n\n".join([default_constructor[0], rule_of_five[0], protobuf_constructor[0], equality_operator[0]])
        )
        constructor_impl = "\n\n".join([default_constructor[1], protobuf_constructor[1], equality_operator[1]])
        converter_headers = indent("\n\n".join([protobuf_converter[0]]))
        converter_impl = "\n\n".join([protobuf_converter[1]])

        header_template = dedent(
            """\
            struct {name} : public ::message::MessageBase<{name}> {{
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
            }};"""
        )

        impl_template = dedent(
            """\
            // Constructors
            {constructors}

            // Converters
            {converters}

            // Subenums
            {enums}

            // Submessages
            {submessages}"""
        )

        python_template = dedent(
            """\
            // Local scope for this message
            {{
                // Use our context and assign a new one to a shadow
                auto shadow = pybind11::class_<{fqn}, std::shared_ptr<{fqn}>>(context, "{name}");

                // Shadow our context with our new context and declare our subclasses
                auto& context = shadow;

            {enums}

            {submessages}

                // Declare the functions on our class (which may use the ones in the subclasses)
                context
            {constructor}
            {message_members};
                context.def_static("include_path", [] {{
                    return "{include_path}";
                }});

                // Build our emitter function that is used to emit this object
                context.def("_emit", [] ({fqn}& msg, pybind11::capsule capsule) {{
                    // Extract our reactor from the capsule
                    NUClear::Reactor* reactor = capsule;

                    // Do the emit
                    reactor->powerplant.emit_shared<NUClear::dsl::word::emit::Local>(msg.shared_from_this());
                }});
            }}"""
        )

        python_constructor_args = ["{}& self".format(self.fqn.replace(".", "::"))]
        python_constructor_args.extend(["{} const& {}".format(t.cpp_type, t.name) for t in self.fields])
        python_members = "\n".join(
            '.def_readwrite("{field}", &{fqn}::{field})'.format(field=f.name, fqn=self.fqn.replace(".", "::"))
            for f in self.fields
        )
        python_constructor_default_args = [""]
        python_constructor_default_args.extend(
            [
                'pybind11::arg("{}") = {}'.format(
                    t.name, t.default_value if t.default_value else "{}()".format(t.cpp_type)
                )
                for t in self.fields
            ]
        )

        python_constructor = dedent(
            """\
            .def("__init__", [] ({args}) {{
                new (&self) {name}({vars});
            }}{default_args})"""
        ).format(
            name=self.fqn.replace(".", "::"),
            args=", ".join(python_constructor_args),
            vars=", ".join("{}".format(t.name) for t in self.fields),
            default_args=", ".join(python_constructor_default_args),
        )

        return (
            header_template.format(
                name=self.name,
                enums=enum_headers,
                submessages=submessage_headers,
                constructors=constructor_headers,
                protobuf_type=protobuf_type,
                converters=converter_headers,
                fields=fields,
            ),
            impl_template.format(
                constructors=constructor_impl, converters=converter_impl, enums=enum_impls, submessages=submessage_impls
            ),
            python_template.format(
                constructor=indent(python_constructor, 8),
                message_members=indent(python_members, 8),
                include_path=self.include_path,
                fqn=self.fqn.replace(".", "::"),
                name=self.name,
                submessages=indent(submessage_python),
                enums=indent(enum_python),
            ),
        )
