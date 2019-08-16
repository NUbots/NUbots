#!/bin/sh
set -e

# Make a directory to out of source build in that's in the container
code_dir=$(pwd)
mkdir /home/nubots/code-build
cd /home/nubots/code-build

# Build the code
cmake ${code_dir} -GNinja
ninja
