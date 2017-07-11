#!/bin/bash

export PATH=/usr/lib/ccache:$PATH
/usr/bin/python3 nuclear/b platform select $PLATFORM
cd build
ninja
