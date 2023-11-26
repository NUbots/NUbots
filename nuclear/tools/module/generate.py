#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2015 NUbots
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
import sys
import textwrap

import b


def register(command):

    # Module help
    command.description = "Generate a new NUClear Roles module at the provided location"

    # Module subcommands
    command.add_argument("path", metavar="path", help="a path to the new module (from the module directory)")


def run(path, **kwargs):
    # Try to get our actual module directory from the cmake cache
    if "NUCLEAR_MODULE_DIR" in b.cmake_cache:
        module_path = os.path.join(b.source_dir, b.cmake_cache["NUCLEAR_MODULE_DIR"])
    else:
        sys.stderr.write("Warning: the system couldn't find the real module directory. Defaulting to module\n")
        module_path = "module"

    # Calculate all of our file paths
    path = os.path.join(module_path, path)
    src_path = os.path.join(path, "src")
    module_name = os.path.split(path)[-1]

    # Check if the path already exists
    if os.path.exists(path):
        sys.stderr.write("The path provided already exists.\n")
        sys.stderr.write("Module generation aborted.\n")
        sys.exit(1)

    print("Module directory", module_path)
    print("Creating directories")

    # Create the required directories
    os.makedirs(path)
    print("\t", path)
    os.makedirs(src_path)
    print("\t", src_path)

    # Split our provided path
    parts = ["module"] + os.path.relpath(path, module_path).split(os.sep)

    print("Generating files")

    # Write all of our files
    with open(os.path.join(path, "CMakeLists.txt"), "w") as output:
        output.write(generate_cmake(parts))
        print("\t", os.path.join(path, "CMakeLists.txt"))

    with open(os.path.join(path, "README.md"), "w") as output:
        output.write(generate_readme(parts))
        print("\t", os.path.join(src_path, "README.md"))

    with open(os.path.join(src_path, "{}.hpp".format(module_name)), "w") as output:
        output.write(generate_header(parts))
        print("\t", os.path.join(src_path, "{}.hpp".format(module_name)))

    with open(os.path.join(src_path, "{}.cpp".format(module_name)), "w") as output:
        output.write(generate_cpp(parts))
        print("\t", os.path.join(src_path, "{}.cpp".format(module_name)))


def generate_cmake(parts):
    return textwrap.dedent(
        """\
        # Build our NUClear module
        NUCLEAR_MODULE()
        """
    )


def generate_header(parts):
    template = textwrap.dedent(
        """\
        #ifndef {define}
        #define {define}

        #include <nuclear>

        {openNamespace}

            class {className} : public NUClear::Reactor {{

            public:
                /// @brief Called by the powerplant to build and setup the {className} reactor.
                explicit {className}(std::unique_ptr<NUClear::Environment> environment);
            }};

        {closeNamespace}

        #endif  // {define}
        """
    )

    return template.format(
        define="{}_HPP".format("_".join([p.upper() for p in parts])),
        className=parts[-1],
        openNamespace="\n".join(["namespace {} {{".format(x) for x in parts[:-1]]),
        closeNamespace="\n".join("}" * (len(parts) - 1)),
    )


def generate_cpp(parts):
    template = textwrap.dedent(
        """\
        #include "{className}.hpp"

        {openNamespace}

            {className}::{className}(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)) {{
            }}
        {closeNamespace}
        """
    )

    return template.format(
        className=parts[-1],
        openNamespace="\n".join(["namespace {} {{".format(x) for x in parts[:-1]]),
        closeNamespace="\n".join(["}" for x in parts[:-1]]),
    )


def generate_readme(parts):
    template = textwrap.dedent(
        """\
        {className}
        {classNameTitle}

        ## Description


        ## Usage


        ## Emits


        ## Dependencies

        """
    )

    return template.format(
        className=parts[-1], classNameTitle=len(parts[-1]) * "=", closeNamespace="\n".join(["}" for x in parts[:-1]])
    )
