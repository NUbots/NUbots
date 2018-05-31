#!/usr/bin/env python3

from generator.textutil import indent, dedent
from generator.Enum import Enum
from generator.Message import Message
import re


class File:

    def __init__(self, f, base_file):

        # Store relevant information from our class in our object
        self.package = f.package
        self.name = f.name
        self.base_file = base_file
        self.include_path = "{}.h".format(base_file)
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
        enum_python = ('\n\n'.join([e[2] for e in enums]))

        # Generate our enums c++
        messages = [m.generate_cpp() for m in self.messages]
        message_headers = indent('\n\n'.join([m[0] for m in messages]))
        message_impls = ('\n\n'.join([m[1] for m in messages]))
        message_python = ('\n\n'.join([m[2] for m in messages]))

        # By default include some useful headers
        # yapf: disable
        includes = {
            '1<cstdint>',
            '2<string>',
            '2<array>',
            '2<exception>',
            '2<map>',
            '2<memory>',
            '2<vector>',
            '4"{}"'.format(self.name[:-6] + '.pb.h'),
            '5"message/MessageBase.h"'
        }
        # yapf: enable

        # We use a dirty hack here of putting a priority on each header
        # to make the includes be in a better order
        for d in self.dependencies:
            if d in ['Vector.proto', 'Matrix.proto']:
                includes.add('4"message/conversion/proto_matrix.h"')
            elif d in ['Neutron.proto']:
                pass  # We don't need to do anything for these ones
            elif d in ['google/protobuf/timestamp.proto', 'google/protobuf/duration.proto']:
                includes.add('4"message/conversion/proto_time.h"')
            else:
                includes.add('4"{}"'.format(d[:-6] + '.h'))

        # Don't forget to remove the first character
        includes = '\n'.join(['#include {}'.format(i[1:]) for i in sorted(list(includes))])

        header_template = dedent(
            """\
            #ifndef {define}
            #define {define}

            {includes}

            {open_namespace}

                // Enum Definitions
            {enums}
                // Message Definitions
            {messages}

            {close_namespace}

            #endif  // {define}
            """
        )

        impl_template = dedent(
            """\
            {include}

            // Enum Implementations
            {enums}

            // Message Implementations
            {messages}
            """
        )

        python_template = dedent(
            """\
            #include <pybind11/pybind11.h>
            #include <pybind11/complex.h>
            #include <pybind11/stl.h>
            #include <pybind11/chrono.h>
            #include <pybind11/operators.h>
            #include <pybind11/eigen.h>

            {include}

            void init_{filename}(pybind11::module& module) {{

                // Go down to our submodule as required as context
                pybind11::module context = module{submodules};

            {enums}

            {messages}
            }}
            """
        )

        python_submodules = ''.join('.def_submodule("{}")'.format(m) for m in self.fqn.split('.')[2:])

        return header_template.format(
            define=define,
            includes=includes,
            open_namespace=ns_open,
            enums=enum_headers,
            messages=message_headers,
            close_namespace=ns_close
        ), impl_template.format(
            include='#include "{}"'.format(self.name[:-6] + '.h'), enums=enum_impls, messages=message_impls
        ), python_template.format(
            include='#include "{}"'.format(self.name[:-6] + '.h'),
            messages=indent(message_python),
            enums=indent(enum_python),
            filename=re.sub(r'[^A-Za-z0-9]', '_', self.name),
            submodules=python_submodules
        )
