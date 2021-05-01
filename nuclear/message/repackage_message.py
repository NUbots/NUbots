#!/usr/bin/env python3

import re
import sys

input_file = sys.argv[1]
output_file = sys.argv[2]

with open(input_file, "r") as f:
    code = f.read()

with open(output_file, "w") as f:
    f.write(re.sub(r"package\s+message", "package protobuf.message", code))
