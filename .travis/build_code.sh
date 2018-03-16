#!/bin/bash

# Define environment variables
PREFIX=/nubots/toolchain

export LD_LIBRARY_PATH="$PREFIX/lib"
export PATH="$PREFIX/bin:/usr/lib/ccache:$PATH"
export PKG_CONFIG_PATH="$PREFIX/lib/pkgconfig"
export CMAKE_PREFIX_PATH="$PREFIX"

/nubots/toolchain/bin/python3 nuclear/b platform select $PLATFORM
cd build
ninja
