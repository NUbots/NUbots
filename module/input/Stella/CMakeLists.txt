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

find_package(OpenCV REQUIRED)

# Add Stella + FBoW include paths
include_directories(SYSTEM
    /usr/local/include
    /usr/local/include/opencv4
    /usr/local/include/eigen3
)

# Link to the Stella + FBoW shared libraries
link_directories(
    /usr/local/lib
)

# Add socket_publisher include path
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/socket_publisher/include
    ${CMAKE_CURRENT_BINARY_DIR}/src/socket_publisher
)

# Add protobuf include directories
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

# Build the socket_publisher library first
add_subdirectory(src/socket_publisher)

# Define your NUClear module
nuclear_module(LIBRARIES
    ${OpenCV_LIBRARIES}
    stella_vslam
    fbow
    socket_publisher
)
