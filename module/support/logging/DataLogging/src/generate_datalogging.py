#!/usr/bin/env python3

import os
import pkgutil
import sys
from textwrap import dedent

import google.protobuf.message

if __name__ == "__main__":
    shared_folder = sys.argv[1]
    cpp_file = sys.argv[2]
    yaml_file = sys.argv[3]

    # Load all our protocol buffer files as modules into this file
    includes = []
    sys.path.append(shared_folder)
    for dir_name, subdir, files in os.walk(shared_folder):
        modules = pkgutil.iter_modules(path=[dir_name])
        for loader, module_name, ispkg in modules:
            if module_name.endswith("pb2"):

                # Work out what header file this came from
                include = os.path.join(os.path.relpath(dir_name, shared_folder), "{}.h".format(module_name[:-4]))

                # If it's one of ours include it
                if include.startswith("message"):
                    includes.append(include)

                # Load our protobuf module
                loader.find_module(module_name).load_module(module_name)

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
        #include "DataLogging.h"

        #include <nuclear>
        #include <fmt/format.h>

        {includes}

        namespace module {{
        namespace support {{
            namespace logging {{

                template <typename T>
                std::unique_ptr<DataLogging::DataLog> log_encode(const T& data) {{

                    auto log = std::make_unique<DataLogging::DataLog>();

                    // We get the timestamp here from the time the message was emitted
                    // (or now if we can't access the current reaction)
                    auto task = NUClear::threading::ReactionTask::get_current_task();
                    log->timestamp = task ? task->stats ? task->stats->emitted : NUClear::clock::now() : NUClear::clock::now();

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
            }}  // namespace logging
        }}  // namespace support
        }}  // namespace module\n"""
    )

    # Work out our includes
    includes = ['#include "{}"'.format(i) for i in includes]

    # Make our recording handles
    handle_template = '            if (name == "{0}") return on<Trigger<{1}>>().then([this](const {1}& d) {{ emit(log_encode(d)); }});'
    handles = [handle_template.format(m, m.replace(".", "::")) for m in sorted(messages)]

    with open(cpp_file, "w") as f:
        f.write(source.format(includes="\n".join(includes), record_handles="\n".join(handles)))

    # Now generate our yaml file
    yaml_template = dedent(
        """\
        output:
          directory: log
          split_size: 5 * GiB

        messages:
        {messages}
    """
    )

    yaml_keys = ["  {}: false".format(m) for m in sorted(messages)]

    # and write it out
    with open(yaml_file, "w") as f:
        f.write(yaml_template.format(messages="\n".join(yaml_keys)))
