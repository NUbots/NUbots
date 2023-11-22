#[[
MIT License

Copyright (c) 2022 NUbots

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

# Silence warnings about target_sources converting relative paths to abolutes
cmake_policy(SET CMP0076 NEW)

find_package(Catch2 REQUIRED)

add_executable(TestNUClearRoles)

# Add all of the tests
add_subdirectory(message)
target_link_libraries(TestNUClearRoles PUBLIC nuclear::message)
target_link_libraries(TestNUClearRoles PUBLIC Catch2::Catch2WithMain)

set_property(TARGET TestNUClearRoles PROPERTY FOLDER "nuclear/tests")

# Add the test
add_test(
  NAME TestNUClearRoles
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/TestNUClearRoles
)
