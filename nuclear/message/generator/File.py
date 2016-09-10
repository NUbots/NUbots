from textutil import indent, dedent
from Enum import Enum
from Message import Message


class File:
    def __init__(self, f):

        # Store relevant information from our class in our object
        self.package = f.package
        self.name = f.name
        self.fqn = '.{}'.format(self.package)
        self.dependencies = [d for d in f.dependency]
        self.enums = [Enum(e, self) for e in f.enum_type]
        self.messages = [Message(m, self) for m in f.message_type]

    def generate_cpp(self):

        define = '{}_H'.format('_'.join([s.upper() for s in self.name[:-6].strip().split('/')]))
        parts = self.package.split('.')
        ns_open = '\n'.join(['namespace {} {{'.format(x) for x in parts])
        ns_close = '\n'.join('}' * len(parts))

        # Generate our enums c++
        enums = [e.generate_cpp() for e in self.enums]
        enum_headers = indent('\n\n'.join([e[0] for e in enums]))
        enum_impls = ('\n\n'.join([e[1] for e in enums]))

        # Generate our enums c++
        messages = [m.generate_cpp() for m in self.messages]
        message_headers = indent('\n\n'.join([m[0] for m in messages]))
        message_impls = ('\n\n'.join([m[1] for m in messages]))

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
            if d in ['Vector.proto', 'Matrix.proto']:
                includes.add('4"message/conversion/proto_matrix.h"')
            elif d in ['Transform.proto']:
                includes.add('4"message/conversion/proto_transform.h"')
            elif d in ['google/protobuf/timestamp.proto', 'google/protobuf/duration.proto']:
                includes.add('4"message/conversion/proto_time.h"')
            elif d in ['google/protobuf/struct.proto']:
                includes.add('4"utility/include/proto_struct.h"')
            else:
                includes.add('4"{}"'.format(d[:-6] + '.h'))

        # Don't forget to remove the first character
        includes = '\n'.join(['#include {}'.format(i[1:]) for i in sorted(list(includes))])

        header_template = dedent("""\
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

        impl_template = dedent("""\
            {include}

            // Enum Implementations
            {enums}

            // Message Implementations
            {messages}
        """)

        return header_template.format(
            define=define,
            includes=includes,
            openNamespace=ns_open,
            enums=enum_headers,
            messages=message_headers,
            closeNamespace=ns_close
        ), impl_template.format(
            include='#include "{}"'.format(self.name[:-6] + '.h'),
            enums=enum_impls,
            messages=message_impls
        )
