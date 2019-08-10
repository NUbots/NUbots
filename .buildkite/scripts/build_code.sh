#!/bin/bash

/usr/bin/python3 nuclear/b platform select $PLATFORM
cd build
ninja
