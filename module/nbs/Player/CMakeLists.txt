#[[
MIT License

Copyright (c) 2024 NUbots

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

# Generate our data logging module to handle all the message types
find_package(Python3 REQUIRED)

set(source_file "${CMAKE_CURRENT_BINARY_DIR}/src/register_players.cpp")

# Generate the emitters for each of the message types we know
add_custom_command(
  OUTPUT "${source_file}"
  COMMAND ${Python3_EXECUTABLE} ARGS "${CMAKE_CURRENT_SOURCE_DIR}/src/generate_emitters.py"
          "${PROJECT_BINARY_DIR}/nuclear/message/python" "${source_file}"
  DEPENDS nuclear::message "${CMAKE_CURRENT_SOURCE_DIR}/src/generate_emitters.py"
  COMMENT "Generating Playback system for current messages"
)

# Build our NUClear module
nuclear_module(SOURCES "${source_file}")
