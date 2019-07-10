#!/usr/bin/env python3

import sys
import os

# Get our input and output from the list
input_path = sys.argv[1]
output_path = sys.argv[2]

# Make our output directory if it doesn't exist
try:
    os.makedirs(os.path.dirname(output_path))
except:
    pass

with open(input_path, "r") as input_file:
    with open(output_path, "w") as output_file:

        # Work out what our define will be
        define = os.path.basename(input_path)[:-3].replace(os.sep, "_").upper()
        output = [l.rstrip("\n").replace("\\", "\\\\").replace('"', '\\"') for l in input_file]
        output.append("")

        output = '#define {}_CL "{}"'.format(define, "\\n\\\n".join(output))
        output_file.write(output)
        output_file.write("\n")
