#!/usr/bin/python

import sys
from subprocess import Popen, PIPE

if sys.argv[1]:
    output_file = sys.argv[1];
else:
    print 'You must specify an output file\n';
    sys.exit(1);

if sys.argv[2:]:
    module_files = sys.argv[2:];
else:
    print 'You must specify some input module files\n';
    sys.exit(1);

# Open our output file for writing
with open(output_file, 'w') as file:

    # TODO read the input json files and make a graph in .dot format from them
    file.write('\n'.join(module_files))