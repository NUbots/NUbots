#!/bin/sh
set -e

# Make a directory to out of source build in that's in the container
mkdir build
cd build

# Build the code
cmake .. -GNinja
ninja
