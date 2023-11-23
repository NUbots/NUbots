#[[
MIT License

Copyright (c) 2013 NUbots

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
include(ExternalProject)

# ######################################################################################################################
# CM730 Firmware builds for both Darwin and Igus models
# ######################################################################################################################

set(firmware_source "${CMAKE_CURRENT_BINARY_DIR}/CM730Firmware")
set(firmware_dir "${CMAKE_BINARY_DIR}/firmware")
set(bin_files "${firmware_source}/build/cm730_3cell.bin" "${firmware_source}/build/cm730_4cell.bin"
              "${firmware_source}/build/cm740_3cell.bin" "${firmware_source}/build/cm740_4cell.bin"
)

ExternalProject_Add(
  CM730Firmware
  DOWNLOAD_DIR "${firmware_source}"
  GIT_REPOSITORY "https://github.com/NUbots/CM730"
  GIT_TAG "master"
  PREFIX "${firmware_source}"
  UPDATE_COMMAND ""
  CMAKE_GENERATOR Ninja
  # CMAKE_ARGS -D -DCMAKE_TOOLCHAIN_FILE:STRING="${firmware_source}/CM730.cmake"
  BINARY_DIR "${firmware_source}/build"
  BUILD_COMMAND ninja
  BUILD_BYPRODUCTS ${bin_files}
                   # We have a custom installation step below.
  INSTALL_DIR "${firmware_source}/build"
  INSTALL_COMMAND ""
                  # We dont need to do any testing.
  TEST_COMMAND ""
)

# Build our NUClear module
foreach(bin_file ${bin_files})
  list(APPEND data_files "${bin_file}:firmware")
endforeach(bin_file)

nuclear_module(DATA_FILES ${data_files})
