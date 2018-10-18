#!/bin/bash

# Define environment variables
PREFIX=/nubots/toolchain

export LD_LIBRARY_PATH="$PREFIX/$PLATFORM/lib:$PREFIX/lib"
export PATH="$PREFIX/$PLATFORM/bin:$PREFIX/bin:/usr/lib/ccache:$PATH"
export PKG_CONFIG_PATH="$PREFIX/$PLATFORM/lib/pkgconfig:$PREFIX/lib/pkgconfig"
export CMAKE_PREFIX_PATH="$PREFIX"

which cmake
cmake --version

/nubots/toolchain/bin/python3 nuclear/b platform select $PLATFORM
cd build
ninja
