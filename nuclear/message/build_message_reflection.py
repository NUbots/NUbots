#!/usr/bin/env python3

import os
import pkgutil
import sys

import google.protobuf.message
import xxhash
from generator.textutil import dedent, indent

if __name__ == "__main__":

    python_message_root = sys.argv[1]
    reflection_output_header = sys.argv[2]

    # Load all our protocol buffer files as modules into this file
    includes = []
    sys.path.append(python_message_root)
    for dir_name, subdir, files in os.walk(python_message_root):
        modules = pkgutil.iter_modules(path=[dir_name])
        for loader, module_name, ispkg in modules:
            if module_name.endswith("pb2"):

                # Work out what header file this came from
                include = os.path.join(
                    os.path.relpath(dir_name, python_message_root), "{}.hpp".format(module_name[:-4])
                )

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

    includes = "\n".join('#include "{}"'.format(i) for i in includes)

    cases = "\n".join(
        [
            "case 0x{}: return std::make_unique<Reflector<{}>>();".format(
                xxhash.xxh64(m, seed=0x4E55436C).hexdigest(), "::".join(m.split("."))
            )
            for m in messages
        ]
    )

    output = dedent(
        """\
        #ifndef MESSAGE_REFLECTION_HPP
        #define MESSAGE_REFLECTION_HPP

        #include <cstdint>
        #include <memory>
        #include <string>
        #include <vector>

        {includes}

        namespace message::reflection {{

            template <template <typename> class Reflector>
            std::unique_ptr<Reflector<void>> from_hash(const uint64_t& hash) {{
                switch (hash) {{
        {cases}
                    default: throw std::runtime_error("Unknown message type with hash " + std::to_string(hash));
                }}
            }}

        }}  // namespace message::reflection

        #endif  // MESSAGE_REFLECTION_HPP
        """
    ).format(includes=includes, cases=indent(cases, 12))

    with open(reflection_output_header, "w") as f:
        f.write(output)
