#!/bin/bash

export PATH=/usr/lib/ccache:$PATH
/usr/bin/python3 nuclear/b platform select $PLATFORM -DOpenCL_INCLUDE_DIR=/opt/intel/opencl/include -DOpenCL_LIBRARY=/opt/intel/opencl/libOpenCL.so
cd build
ninja
