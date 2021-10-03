#! /usr/bin/env python3

import hashlib
import os
import subprocess
import sys

# Get the output folder from the arguments
output_folder = sys.argv[1]
clang_tidy_binary = sys.argv[2]

os.makedirs(output_folder, exist_ok=True)

# Hash the remaining arguments as the output file name
output_file = "{}.yaml".format(hashlib.sha256(" ".join(sys.argv[2:]).encode("utf-8")).hexdigest())

# Work out our arguments to call the clang-tidy binary with
args = [clang_tidy_binary, "--export-fixes={}".format(os.path.join(output_folder, output_file)), *sys.argv[3:]]

# Run clang-tidy and pass the return code back to the caller
exit(subprocess.run(args).returncode)
