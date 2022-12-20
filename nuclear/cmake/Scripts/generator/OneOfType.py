#!/usr/bin/env python3

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

    def generate_cpp(self):

        header_template = dedent(
            """\
            class {oneof_name} {{
            public:
                {oneof_name}() : {initialiser_list} {{}}
                {oneof_name}(const {oneof_name}& rhs) noexcept : {initialiser_list}, val_index(rhs.val_index), data(rhs.data) {{}}
                {oneof_name}({oneof_name}&& rhs) noexcept : {initialiser_list}, val_index(std::exchange(rhs.val_index, 0)), data(std::move(rhs.data)) {{}}
                ~{oneof_name}() = default;

                {oneof_name}& operator=(const {oneof_name}& rhs) noexcept {{
                    if (&rhs != this) {{
                        data = rhs.data;
                        val_index = rhs.val_index;
                    }}
                    return *this;
                }}
                {oneof_name}& operator=({oneof_name}&& rhs) noexcept {{
                    data = std::move(rhs.data);
                    val_index = std::exchange(rhs.val_index, 0);
                    return *this;
                }}

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
                    val_index = 0;
                    data.reset();
                }}

            {member_list}

                size_t val_index{{0}};

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

        return (
            header_template.format(
                oneof_name=self.name,
                member_list=indent("\n".join(member_list), 4),
                initialiser_list=", ".join(initialiser_list),
                cases=indent("\n".join(cases), 16),
            ),
            impl_template,
            python_template.format(
                bindings="\n\n".join(python_bindings), fqn=self.fqn.replace(".", "::"), name=self.name
            ),
        )
