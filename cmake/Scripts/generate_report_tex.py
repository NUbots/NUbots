#!/usr/bin/python

import sys

if sys.argv[1]:
    output_file = sys.argv[1];
else:
    print 'You must specify an output file\n';
    sys.exit(1);

if sys.argv[2]:
    header = sys.argv[2];
else:
    print 'You must specify an output file\n';
    sys.exit(1);

if sys.argv[3]:
    footer = sys.argv[3];
else:
    print 'You must specify an output file\n';
    sys.exit(1);

if sys.argv[3:]:
    tex_sources = sys.argv[3:];
else:
    print 'You must specify some report modules\n';
    sys.exit(1);

# Open our output file for writing
with open(output_file, 'w') as file:

    # Put our header into the output file
    with open(header, 'r') as header:
        data = header.read();
        file.write(data);
        file.write("\n");

    # Sort out our modules for inclusion order

    # Loop through our modules sources
    for src in tex_sources:
        with open(src, 'r') as tex:
            data = tex.read();
            file.write(data);
            file.write("\n");



    # Put our footer into the output file
    with open(footer, 'r') as footer:
        data = footer.read();
        file.write(data);
        file.write("\n");
