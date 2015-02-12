#!/usr/bin/python

import sys
from subprocess import Popen, PIPE

if sys.argv[1]:
    input_file = sys.argv[1];
else:
    print 'You must specify an input file\n';
    sys.exit(1);

if sys.argv[2]:
    output_file = sys.argv[2];
else:
    print 'You must specify an output file\n';
    sys.exit(1);

# Open our output file for writing
with open(output_file, 'w') as file:

    # Run nm on the file
    process = Popen(["nm", "-g", "-C", input_file], stdout=PIPE);
    (output, err) = process.communicate();
    exit_code = process.wait();

    # If our nm command failed then exit with the error
    if(exit_code != 0):
        print err;
        exit(exit_code);

    # TODO do some processing to make a json file of the interaces on this file
    file.write(output)