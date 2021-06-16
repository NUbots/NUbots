#!/usr/bin/env python3

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
        #include "DataPlayback.hpp"

        {includes}

        namespace module::support::logging {{

            void DataPlayback::register_players() {{
        {players}
            }}

        }}  // namespace module::support::logging

        """
    )

    # Work out our includes and players
    includes = ['#include "{}"'.format(i) for i in includes]
    players = ["            add_player<{}>();".format(m.replace(".", "::")) for m in messages]

    with open(cpp_file, "w") as f:
        f.write(source.format(includes="\n".join(includes), players="\n".join(players)))
