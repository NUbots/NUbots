#!/usr/bin/python
#
# MIT License
#
# Copyright (c) 2019 NUbots
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

from generator.textutil import dedent, indent

# Get our file we are outputting too
base_file = sys.argv[1]

# Get our root message directory
message_dir = sys.argv[2]

# Get our list of functions we have to call
functions = []
duplicates = []
for dep_file in sys.argv[3:]:
    with open(dep_file) as deps:
        # Extract all dependencies for every message and place them in the list.
        # Make all paths relative to the root mesage directory and remove any unwanted characters.
        # Also remove Matrix.proto, Neutron.proto, and Vector.proto from the list and anything to do with google.
        dependencies = [
            os.path.relpath(s.strip("\\ \n\t"), message_dir).replace("/", "_").replace(".proto", "_proto")
            for s in deps.readlines()
            if not any(exclude in s for exclude in ["google/protobuf", "Matrix.proto", "Neutron.proto", "Vector.proto"])
        ]

        # Finally, remove duplicates. We must keep the first instance of every message in the list.
        for function in dependencies:
            if function not in duplicates:
                duplicates.append(function)
                functions.append(function)

# Write our file
with open(base_file, "w") as f:

    f.write(
        dedent(
            """\
        #include <pybind11/pybind11.h>
        #include <pybind11/complex.h>
        #include <pybind11/stl.h>
        #include <pybind11/eigen.h>

        // Declare our functions (that we know will be made later)
        {function_declarations}

        PYBIND11_PLUGIN(message) {{

            pybind11::module module("message", "NUClear message classes");

            // Initialise each of the modules
        {function_calls}

            return module.ptr();
        }}
        """
        ).format(
            function_declarations="\n".join(
                "void init_message_{}(pybind11::module& module);".format(f) for f in functions
            ),
            function_calls=indent("\n".join("init_message_{}(module);".format(f) for f in functions)),
        )
    )
