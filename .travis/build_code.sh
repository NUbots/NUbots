#!/bin/bash

export PATH=/usr/lib/ccache:$PATH
cat /nubots/toolchain/$PLATFORM.cmake
/usr/bin/python3 nuclear/b platform select $PLATFORM
cd build
ninja
