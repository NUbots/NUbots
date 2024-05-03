#[[
MIT License

Copyright (c) 2019 NUbots

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

# Create the clock library
add_library(nubots_clock STATIC clock.cpp clock.hpp)
target_compile_definitions(nubots_clock PUBLIC NUCLEAR_CUSTOM_CLOCK)
set_target_properties(nubots_clock PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(nubots_clock PUBLIC cxx_std_17)

# Link such that anything that is using the nuclear library will get this library too
target_link_libraries(nuclear_message PUBLIC nubots_clock)
target_link_libraries(nuclear_utility PUBLIC nubots_clock)
