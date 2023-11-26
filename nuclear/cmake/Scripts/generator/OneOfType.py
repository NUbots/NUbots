#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2018 NUbots
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

import stringcase
from generator.Field import Field
from generator.textutil import dedent, indent


class OneOfType:
    def __init__(self, name, fields, context):

        self.package = context.package
        self.name = "OneOf{}".format(stringcase.pascalcase(name))
        self.fqn = "{}.{}".format(context.fqn, self.name)
        self.include_path = context.include_path

        self.fields = [Field(f, context) for f in fields]

        self.enum_values = [(f.name.upper(), f.number) for f in self.fields]

    def generate_cpp(self):

        header_template = dedent(
            """\
            class {oneof_name} {{
            public:
                {oneof_name}() : {initialiser_list} {{}}
                {oneof_name}(const {oneof_name}& rhs) noexcept : {initialiser_list}, val_index(rhs.val_index), value(rhs.value), data(rhs.data) {{}}
                {oneof_name}({oneof_name}&& rhs) noexcept : {initialiser_list}, val_index(std::exchange(rhs.val_index, {default_enum_index})), value(std::exchange(rhs.value, {{Value::{default_enum_value}}})), data(std::move(rhs.data)) {{}}
                ~{oneof_name}() = default;

                {oneof_name}& operator=(const {oneof_name}& rhs) noexcept {{
                    if (&rhs != this) {{
                        data = rhs.data;
                        val_index = rhs.val_index;
                        value = rhs.value;
                    }}
                    return *this;
                }}
                {oneof_name}& operator=({oneof_name}&& rhs) noexcept {{
                    if (&rhs != this) {{
                        data = std::move(rhs.data);
                        val_index = std::exchange(rhs.val_index, {default_enum_index});
                        value = std::exchange(rhs.value, {{Value::{default_enum_value}}});
                    }}
                    return *this;
                }}

                enum Value {{
            {enum_values}
                }};

                template <typename T, size_t index>
                class Proxy {{
                public:
                    Proxy({oneof_name}* oneof) : oneof(oneof) {{}}
                    ~Proxy() = default;
                    Proxy(const Proxy&) = delete;
                    Proxy(Proxy&&) = delete;
                    Proxy& operator=(const Proxy&) = delete;
                    Proxy& operator=(Proxy&&) = delete;

                    template <typename U = T>
                    Proxy& operator=(U&& t) {{
                        oneof->data      = std::make_shared<T>(std::forward<U>(t));
                        oneof->val_index = index;
                        oneof->value = static_cast<Value>(index);
                        return *this;
                    }}

                    bool operator==(const Proxy<T, index>& rhs) const {{ return get() == rhs.get(); }}

                    operator T&() {{ return get(); }}

                    operator const T&() const {{ return get(); }}

                    [[nodiscard]] const T& get() const {{
                        if (oneof->val_index == index) {{
                            return *std::static_pointer_cast<T>(oneof->data);
                        }}

                        throw std::range_error("This is not the one of you were looking for.");
                    }}

                    [[nodiscard]] T& get() {{
                        if (oneof->val_index == index) {{
                            return *std::static_pointer_cast<T>(oneof->data);
                        }}

                        throw std::range_error("This is not the one of you were looking for.");
                    }}

                private:
                    {oneof_name}* oneof;
                }};

                bool operator==(const {oneof_name}& rhs) const {{
                    if (val_index == rhs.val_index) {{
                        switch (val_index) {{
            {cases}
                            default: return false;
                        }}
                    }}
                    return false;
                }}

                void reset() {{
                    val_index = {default_enum_index};
                    value = {{Value::{default_enum_value}}};
                    data.reset();
                }}

            {member_list}

                size_t val_index{{{default_enum_index}}};
                Value value{{Value::{default_enum_value}}};

            private:
                std::shared_ptr<void> data{{nullptr}};
            }};
            """
        )

        impl_template = dedent(
            """\
                        """
        )

        python_template = dedent(
            """\
            // Local scope for this message
            {{
                // Use our context and assign a new one to a shadow
                auto shadow = pybind11::class_<{fqn}, std::shared_ptr<{fqn}>>(context, "{name}");

                // Shadow our context with our new context and declare our subclasses
                auto& context = shadow;

                // Declare the functions on our class (which may use the ones in the subclasses)
                context
                {bindings}
            }}"""
        )

        binding_template = dedent(
            """\
            context.def_property("{field_name}",
                [](const {fqn}& oneof) -> const {type_name}& {{
                    return oneof.{field_name};
                }},
                [](const {fqn}& oneof, const {type_name}& value) -> void {{
                    oneof.{field_name} = value;
                }});
            """
        )

        python_bindings = [
            binding_template.format(field_name=v.name, type_name=v.cpp_type, fqn=self.fqn.replace(".", "::"))
            for v in self.fields
        ]

        member_list = ["Proxy<{}, {}> {};".format(v.cpp_type, v.number, v.name) for v in self.fields]
        initialiser_list = ["{}(this)".format(v.name) for v in self.fields]
        cases = ["case {0}: return {1} == rhs.{1};".format(v.number, v.name) for v in self.fields]
        enum_values = indent("\n".join(["{} = {}".format(v[0], v[1]) for v in self.enum_values]), 8)
        enum_values = ",\n".join([v for v in enum_values.splitlines()])
        default_enum_value = self.enum_values[0][0]
        default_enum_index = self.enum_values[0][1]

        return (
            header_template.format(
                oneof_name=self.name,
                member_list=indent("\n".join(member_list), 4),
                initialiser_list=", ".join(initialiser_list),
                cases=indent("\n".join(cases), 16),
                enum_values=enum_values,
                default_enum_value=default_enum_value,
                default_enum_index=default_enum_index,
            ),
            impl_template,
            python_template.format(
                bindings="\n\n".join(python_bindings), fqn=self.fqn.replace(".", "::"), name=self.name
            ),
        )
