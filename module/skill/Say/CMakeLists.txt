#[[
MIT License

Copyright (c) 2023 NUbots

This file is part of the NUbots codebase.
See https://github.com/NUbots/NUbots for further info.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
]]

# Set the path to Piper directory
set(PIPER_DIR "/openvino/piper" CACHE PATH "Path to the Piper directory")
set(PIPER_LIB_DIR "${PIPER_DIR}/build/pi/lib" CACHE PATH "Path to the Piper library directory")
set(PIPER_INCLUDE_DIR "${PIPER_DIR}/build/pi/include" CACHE PATH "Path to the Piper include directory")

message("PIPER_DIR: ${PIPER_DIR}")
message("PIPER_LIB_DIR: ${PIPER_LIB_DIR}")
message("PIPER_INCLUDE_DIR: ${PIPER_INCLUDE_DIR}")

# Add the include directory globally
include_directories(SYSTEM "${PIPER_INCLUDE_DIR}")
include_directories(SYSTEM "${PIPER_INCLUDE_DIR}/piper-phonemize")

# Set up RPATH for finding libraries at runtime
set(CMAKE_INSTALL_RPATH "${PIPER_LIB_DIR}")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Check if the libraries exist with correct version numbers
if(NOT EXISTS "${PIPER_LIB_DIR}/libpiper_phonemize.so.1.2.0" OR
   NOT EXISTS "${PIPER_LIB_DIR}/libonnxruntime.so.1.14.1" OR
   NOT EXISTS "${PIPER_LIB_DIR}/libespeak-ng.so.1.52.0.1")
    message(WARNING "One or more Piper libraries not found in ${PIPER_LIB_DIR}")
endif()

# Build our NUClear module
nuclear_module(
    INCLUDES
        "${PIPER_INCLUDE_DIR}"
        "${PIPER_INCLUDE_DIR}/piper-phonemize"
    LIBRARIES
        "${PIPER_LIB_DIR}/libpiper_phonemize.so.1.2.0"
        "${PIPER_LIB_DIR}/libonnxruntime.so.1.14.1"
        "${PIPER_LIB_DIR}/libespeak-ng.so.1.52.0.1"
)
