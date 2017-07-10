#!/bin/bash

export PATH=/usr/lib/ccache:$PATH
/usr/bin/python3 ./b platform select $PLATFORM
cd build
ninja
