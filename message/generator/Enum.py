from textutil import indent, dedent


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

    def generate_cpp(self):

        # Make our value pairs
        values = indent('\n'.join(['{} = {}'.format(v[0], v[1]) for v in self.values]), 8)
        values = ',\n'.join([v for v in values.splitlines()])

        # Make our switch statement pairs
        switches = indent('\n'.join(['case Value::{}: return "{}";'.format(v[0], v[0]) for v in self.values]), 8)

        # Make our if chain
        if_chain = indent('\n'.join(['if (str == "{}") value = Value::{};'.format(v[0], v[0]) for v in self.values]))

        # Get our default value
        default_value = dict([reversed(v) for v in self.values])[0]

        # Make our fancy enums
        header_template = dedent("""\
            struct {name} {{
                enum Value {{
            {values}
                }};
                Value value;

                // Constructors
                {name}();

                {name}(const int& v);

                {name}(const Value& value);

                {name}(const std::string& str);

                {name}(const {protobuf_name}& p);

                // Operators
                bool operator <(const {name}& other) const;

                bool operator <(const {name}::Value& other) const;

                bool operator ==(const {name}& other) const;

                bool operator ==(const {name}::Value& other) const;

                // Conversions
                operator Value() const;

                operator int() const;

                operator std::string() const;

                operator {protobuf_name}() const;
            }};
            """)

        impl_template = dedent("""\
            {fqn}::{name}() : value(Value::{default_value}) {{}}

            {fqn}::{name}(const int& v) : value(static_cast<Value>(v)) {{}}

            {fqn}::{name}(const Value& value) : value(value) {{}}

            {fqn}::{name}(const std::string& str) {{
            {if_chain}
                throw std::runtime_error("String did not match any enum for {name}");
            }}

            {fqn}::{name}(const {protobuf_name}& p) {{
                value = static_cast<Value>(p);
            }}

            bool {fqn}::operator <(const {name}& other) const {{
                return value < other.value;
            }}

            bool {fqn}::operator <(const {name}::Value& other) const {{
                return value < other;
            }}

            bool {fqn}::operator ==(const {name}& other) const {{
                return value == other.value;
            }}

            bool {fqn}::operator ==(const {name}::Value& other) const {{
                return value == other;
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
            """)

        return header_template.format(
            name=self.name,
            protobuf_name='::'.join(('.protobuf' + self.fqn).split('.')),
            values=values
        ), impl_template.format(
            fqn='::'.join(self.fqn.split('.')),
            name=self.name,
            protobuf_name='::'.join(('.protobuf' + self.fqn).split('.')),
            default_value=default_value,
            if_chain=if_chain,
            switches=switches
        ),

