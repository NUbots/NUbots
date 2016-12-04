#!/usr/bin/python

import os
import sys
import re

input_file = sys.argv[1]
output_dir = sys.argv[2]

with open(input_file, 'r') as f:
    code = f.read()

regex = re.compile(r'package\s+message')

with open('{}/{}'.format(output_dir, os.path.basename(input_file)), 'w') as f:
    f.write(regex.sub('package protobuf.message', code))
