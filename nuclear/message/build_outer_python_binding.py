#!/usr/bin/python
import os
import sys

from generator.textutil import dedent, indent

# Get our file we are outputting to
base_file = sys.argv[1]

# Get our list of functions we have to call
functions = []
message_dir = sys.argv[2].strip()
abs_neutron_names = sys.argv[3].strip().split(" ")

rel_neutron_names = []
for name in abs_neutron_names:
    name = name.replace(message_dir, "")
    name = name.replace("/", "_")
    rel_neutron_names.append(name.replace(message_dir, ""))

# Write our file
with open(base_file, "w") as f:

    # TODO: {function_declarations} and {function_calls} are empty atm. These need to be fixed.
    f.write(
        dedent(
            """\
        #include <pybind11/pybind11.h>
        #include <pybind11/complex.h>
        #include <pybind11/stl.h>
        #include <pybind11/eigen.h>

        // Declare our functions (that we know will be made later)
        {function_declarations}

        PYBIND11_MODULE(message, module) {{

            // Initialise each of the modules
        {function_calls}

        }}
        """
        ).format(
            function_declarations="\n".join(
                "void init{}_proto(pybind11::module& module);".format(f) for f in rel_neutron_names
            ),
            function_calls=indent("\n".join("init{}_proto(module);".format(f) for f in rel_neutron_names)),
        )
    )
