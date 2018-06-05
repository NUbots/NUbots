#!/usr/bin/python
import sys
import os
from generator.textutil import indent, dedent

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
        dependencies = [os.path.relpath(s.strip('\\ \n\t'), message_dir).replace('/', '_').replace('.proto', '_proto') for s in deps.readlines()
                if not any(exclude in s for exclude in ['google/protobuf', 'Matrix.proto', 'Neutron.proto', 'Vector.proto'])]

        # Finally, remove duplicates. We must keep the first instance of every message in the list.
        for function in dependencies:
            if function not in duplicates:
                duplicates.append(function)
                functions.append(function)

# Write our file
with open(base_file, 'w') as f:

    f.write(dedent("""\
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
        """).format(
            function_declarations='\n'.join('void init_message_{}(pybind11::module& module);'.format(f) for f in functions),
            function_calls=indent('\n'.join('init_message_{}(module);'.format(f) for f in functions)),
        ))

