#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2017 NUbots
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
                    loader.find_module(fqdn).load_module(fqdn)

    # Now that we've imported them all get all the subclasses of protobuf message
    messages = set()
    for message in google.protobuf.message.Message.__subclasses__():
        # Work out our original protobuf type
        pb_type = ".".join(message.DESCRIPTOR.full_name.split(".")[1:])

        # Only include our own messages
        if pb_type.startswith("message.") and not message.DESCRIPTOR.GetOptions().map_entry:
            messages.add(pb_type)

    messages = list(messages)

    # The base of our source file we will be filling in
    source = dedent(
        """\
        #include "DataLogging.hpp"

        #include "utility/nbs/get_timestamp.hpp"
        #include "utility/nbs/get_id.hpp"

        #include <nuclear>
        #include <fmt/format.h>

        {includes}

        namespace module::support::logging {{

            template <typename T>
            std::unique_ptr<DataLogging::DataLog> log_encode(const T& data) {{

                auto log = std::make_unique<DataLogging::DataLog>();

                // We get the timestamp here from the time the message was emitted
                // (or now if we can't access the current reaction)
                const auto* task = NUClear::threading::ReactionTask::get_current_task();
                log->timestamp   = task ? task->stats ? task->stats->emitted : NUClear::clock::now() : NUClear::clock::now();

                // Get the data from the message to be used in the index
                log->message_timestamp = utility::nbs::get_timestamp(log->timestamp, data);
                log->id                = utility::nbs::get_id(data);

                // Serialise the data and get the hash for it
                log->data = NUClear::util::serialise::Serialise<T>::serialise(data);
                log->hash = NUClear::util::serialise::Serialise<T>::hash();

                return log;
            }}

            NUClear::threading::ReactionHandle DataLogging::activate_recorder(const std::string& name) {{
                // Find the reaction handle we are activating
        {record_handles}

                // We didn't find the one we were looking for, throw an error
                throw std::runtime_error(fmt::format("Could not find message type {{}}", name));
            }}
        }}  // namespace module::support::logging\n"""
    )

    # Work out our includes
    includes = ['#include "{}"'.format(i) for i in includes]

    # Make our recording handles
    handle_template = '        if (name == "{0}") {{ return on<Trigger<{1}>>().then([this](const {1}& d) {{ emit(log_encode(d)); }}); }}'
    handles = [handle_template.format(m, m.replace(".", "::")) for m in sorted(messages)]

    with open(cpp_file, "w") as f:
        f.write(source.format(includes="\n".join(includes), record_handles="\n".join(handles)))
