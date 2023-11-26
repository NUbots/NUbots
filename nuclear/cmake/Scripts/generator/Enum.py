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

from generator.textutil import dedent, indent


class Enum:
    def __init__(self, e, context):
        self.package = context.package
        self.name = e.name
        self.fqn = "{}.{}".format(context.fqn, self.name)
        self.values = [(v.name, v.number) for v in e.value]
        self.include_path = context.include_path

    def generate_cpp(self):

        # Make our value pairs
        values = indent("\n".join(["{} = {}".format(v[0], v[1]) for v in self.values]), 8)
        values = ",\n".join([v for v in values.splitlines()])

        scope_name = "_".join(self.fqn.split("."))

        # Make our switch statement pairs
        switches = indent("\n".join(['case Value::{}: return "{}";'.format(v[0], v[0]) for v in self.values]), 8)

        # Make our if chain
        if_chain = indent(
            "\nelse ".join(['if (str == "{}") {{ value = Value::{}; }}'.format(v[0], v[0]) for v in self.values])
        )

        # Get our default value
        default_value = dict([reversed(v) for v in self.values])[0]

        # Make our fancy enums
        header_template = dedent(
            """\
            struct {name} : public ::message::MessageBase<{name}> {{
                enum Value {{
            {values}
                }};
                Value value{{Value::{default_value}}};

                static constexpr size_t MAX_VALUE = {max_value};

                // Constructors
                {name}();

                {name}(int const& v);

                {name}(Value const& value);

                {name}(std::string const& str);

                {name}({protobuf_name} const& p);

                // Operators
                bool operator <({name} const& other) const;

                bool operator >({name} const& other) const;

                bool operator <=({name} const& other) const;

                bool operator >=({name} const& other) const;

                bool operator ==({name} const& other) const;

                bool operator !=({name} const& other) const;

                bool operator <({name}::Value const& other) const;

                bool operator >({name}::Value const& other) const;

                bool operator <=({name}::Value const& other) const;

                bool operator >=({name}::Value const& other) const;

                bool operator ==({name}::Value const& other) const;

                bool operator !=({name}::Value const& other) const;

                // Conversions
                operator Value() const;

                operator int() const;

                operator std::string() const;

                operator {protobuf_name}() const;

                friend std::ostream& operator<< (std::ostream& out, const {name}& val);
            }};"""
        )

        impl_template = dedent(
            """\
            using T{scope_name} = {fqn};

            {fqn}::{name}() = default;

            {fqn}::{name}(int const& v) : value(static_cast<Value>(v)) {{}}

            {fqn}::{name}(Value const& value) : value(value) {{}}

            {fqn}::{name}(std::string const& str) {{
            {if_chain}
                else {{ throw std::runtime_error("String " + str + " did not match any enum for {name}"); }}
            }}

            {fqn}::{name}({protobuf_name} const& p) : value(static_cast<Value>(p)) {{}}

            bool {fqn}::operator <({name} const& other) const {{
                return value < other.value;
            }}

            bool {fqn}::operator >({name} const& other) const {{
                return value > other.value;
            }}

            bool {fqn}::operator <=({name} const& other) const {{
                return value <= other.value;
            }}

            bool {fqn}::operator >=({name} const& other) const {{
                return value >= other.value;
            }}

            bool {fqn}::operator ==({name} const& other) const {{
                return value == other.value;
            }}

            bool {fqn}::operator !=({name} const& other) const {{
                return value != other.value;
            }}

            bool {fqn}::operator <({name}::Value const& other) const {{
                return value < other;
            }}

            bool {fqn}::operator >({name}::Value const& other) const {{
                return value > other;
            }}

            bool {fqn}::operator <=({name}::Value const& other) const {{
                return value <= other;
            }}

            bool {fqn}::operator >=({name}::Value const& other) const {{
                return value >= other;
            }}

            bool {fqn}::operator ==({name}::Value const& other) const {{
                return value == other;
            }}

            bool {fqn}::operator !=({name}::Value const& other) const {{
                return value != other;
            }}

            {fqn}::operator Value() const {{
                return value;
            }}

            {fqn}::operator int() const {{
                return value;
            }}

            {fqn}::operator std::string() const {{
                switch(value) {{
            {switches}
                    default:
                        throw std::runtime_error("enum {name}'s value is corrupt, unknown value stored");
                }}
            }}

            {fqn}::operator {protobuf_name}() const {{
                return static_cast<{protobuf_name}>(value);
            }}

            namespace {namespace} {{
                std::ostream& operator<< (std::ostream& out, const {fqn}& val) {{
                    return out << static_cast<std::string>(val);
                }}
            }}

            """
        )

        python_template = dedent(
            """\
            // Local scope for this enum
            {{
                auto enumclass = pybind11::class_<{fqn}, std::shared_ptr<{fqn}>>(context, "{name}")
                    .def(pybind11::init<>())
                    .def(pybind11::init<int const&>())
                    .def(pybind11::init<{fqn}::Value const&>())
                    .def(pybind11::init<std::string const&>())
                    .def(pybind11::self < pybind11::self)
                    .def(pybind11::self > pybind11::self)
                    .def(pybind11::self <= pybind11::self)
                    .def(pybind11::self >= pybind11::self)
                    .def(pybind11::self == pybind11::self)
                    .def(pybind11::self < {fqn}::Value())
                    .def(pybind11::self > {fqn}::Value())
                    .def(pybind11::self <= {fqn}::Value())
                    .def(pybind11::self >= {fqn}::Value())
                    .def(pybind11::self == {fqn}::Value())
                    .def_static("include_path", [] {{
                        return "{include_path}";
                    }})
                    .def("_emit", [] ({fqn}& msg, pybind11::capsule capsule) {{
                        // Extract our reactor from the capsule
                        NUClear::Reactor* reactor = capsule;

                        // Do the emit
                        reactor->powerplant.emit_shared<NUClear::dsl::word::emit::Local>(msg.shared_from_this());
                    }});

                pybind11::enum_<{fqn}::Value>(enumclass, "Value")
            {value_list}
                    .export_values();
            }}"""
        )

        return (
            header_template.format(
                name=self.name,
                protobuf_name="::".join((".protobuf" + self.fqn).split(".")),
                values=values,
                default_value=default_value,
                max_value=max([v[1] for v in self.values]),
            ),
            impl_template.format(
                fqn="::".join(self.fqn.split(".")),
                namespace="::".join(self.package.split(".")),
                name=self.name,
                protobuf_name="::".join((".protobuf" + self.fqn).split(".")),
                if_chain=if_chain,
                switches=switches,
                scope_name=scope_name,
            ),
            python_template.format(
                fqn="::".join(self.fqn.split(".")),
                name=self.name,
                include_path=self.include_path,
                value_list=indent(
                    "\n".join(
                        '.value("{name}", {fqn}::{name})'.format(name=v[0], fqn=self.fqn.replace(".", "::"))
                        for v in self.values
                    ),
                    8,
                ),
            ),
        )
