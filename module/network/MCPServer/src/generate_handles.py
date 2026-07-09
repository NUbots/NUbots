#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2026 NUbots
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

import importlib
import os
import pkgutil
import sys
from textwrap import dedent

import google.protobuf.message

if __name__ == "__main__":
    shared_folder = sys.argv[1]
    cpp_file = sys.argv[2]

    # Load all our protocol buffer files as modules into this file
    includes = []
    sys.path.append(shared_folder)
    for dir_name, subdir, files in os.walk(shared_folder):
        modules = pkgutil.iter_modules(path=[dir_name])
        for loader, module_name, ispkg in modules:
            if module_name.endswith("pb2"):
                # Work out what header file this came from
                include = os.path.join(os.path.relpath(dir_name, shared_folder), "{}.hpp".format(module_name[:-4]))

                # If it's one of ours include it
                if include.startswith("message"):
                    includes.append(include)

                # Load our protobuf module
                fqdn = os.path.normpath(os.path.join(os.path.relpath(dir_name, shared_folder), module_name)).replace(
                    os.sep, "."
                )
                if fqdn not in sys.modules:
                    importlib.import_module(fqdn)

    # Now that we've imported them all get all the subclasses of protobuf message
    # For each message keep both the neutron name (message.*) and the protobuf name used for descriptor lookup,
    # and collect the descriptors of every file the messages need, dependencies first
    messages = {}
    file_descriptors = {}  # name -> serialized FileDescriptorProto, insertion order is dependency order

    def add_file(fd):
        if fd.name in file_descriptors:
            return
        for dep in fd.dependencies:
            add_file(dep)
        file_descriptors[fd.name] = fd.serialized_pb

    for message in google.protobuf.message.Message.__subclasses__():
        # Work out our original protobuf type
        full_name = message.DESCRIPTOR.full_name
        pb_type = ".".join(full_name.split(".")[1:])

        # Only include our own messages
        if pb_type.startswith("message.") and not message.DESCRIPTOR.GetOptions().map_entry:
            messages[pb_type] = full_name
            add_file(message.DESCRIPTOR.file)

    # The base of our source file we will be filling in
    source = dedent(
        """\
        #include "MCPServer.hpp"

        #include <google/protobuf/descriptor.pb.h>

        {includes}

        namespace module::network {{

            void MCPServer::register_handles() {{
        {handles}
            }}

            namespace {{
        {descriptor_data}

                struct FileDescriptorData {{
                    const unsigned char* data;
                    size_t size;
                }};

                const FileDescriptorData file_descriptor_data[] = {{
        {descriptor_list}
                }};
            }}  // namespace

            void MCPServer::register_descriptors() {{
                for (const auto& fd : file_descriptor_data) {{
                    google::protobuf::FileDescriptorProto fdp;
                    if (fdp.ParseFromArray(fd.data, static_cast<int>(fd.size))) {{
                        descriptor_pool.BuildFile(fdp);
                    }}
                }}
            }}

        }}  // namespace module::network

        """
    )

    def cpp_byte_array(name, data):
        # Emit the bytes as decimal integers, wrapped for readability
        numbers = ",".join(str(b) for b in data)
        lines = []
        while len(numbers) > 100:
            split = numbers.rfind(",", 0, 100) + 1
            lines.append(numbers[:split])
            numbers = numbers[split:]
        lines.append(numbers)
        body = "\n                    ".join(lines)
        return "        const unsigned char {}[] = {{\n                    {}}};".format(name, body)

    # Work out our includes, handles and descriptors
    includes = ['#include "{}"'.format(i) for i in includes]
    handles = [
        '        add_handle<{}>("{}", "{}");'.format(m.replace(".", "::"), m, full_name)
        for m, full_name in sorted(messages.items())
    ]
    descriptor_data = [
        cpp_byte_array("fd_{}".format(i), data) for i, data in enumerate(file_descriptors.values())
    ]
    descriptor_list = [
        "            {{fd_{0}, sizeof(fd_{0})}},".format(i) for i in range(len(file_descriptors))
    ]

    with open(cpp_file, "w") as f:
        f.write(
            source.format(
                includes="\n".join(includes),
                handles="\n".join(handles),
                descriptor_data="\n".join(descriptor_data),
                descriptor_list="\n".join(descriptor_list),
            )
        )
