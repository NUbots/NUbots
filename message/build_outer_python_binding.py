#!/usr/bin/python
import sys
import re
from generator.textutil import indent, dedent

# Get our file we are outputting too
base_file = sys.argv[1]

# Get our list of functions we have to call
functions = [re.sub(r'[^A-Za-z0-9]', '_', f) for f in sys.argv[2:]]

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
