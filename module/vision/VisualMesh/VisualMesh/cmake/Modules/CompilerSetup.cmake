# Copyright (C) 2013      Trent Houliston <trent@houliston.me>, Jake Woods <jake.f.woods@gmail.com>
#               2014-2017 Trent Houliston <trent@houliston.me>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
# documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
# Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
# WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
# OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

##############
# GENERATORS #
###############

# XCode support
IF(CMAKE_GENERATOR MATCHES Xcode)
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++14")
ENDIF()

###############
#  COMPILERS  #
###############

# We use position independent code
SET(CMAKE_POSITION_INDEPENDENT_CODE ON)

# GNU Compiler
IF(CMAKE_CXX_COMPILER_ID MATCHES GNU)

    # GCC Must be version 4.8 or greater for used features
    IF(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.8)
        MESSAGE(FATAL_ERROR "${PROJECT_NAME} requires g++ 4.8 or greater.")
    ENDIF()

    # Enable colours on g++ 4.9 or greater
    IF(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9 OR CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 4.9)
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always")
    ENDIF()

    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -ftemplate-backtrace-limit=0 -Wall -Wpedantic")

# Clang Compiler
ELSEIF(CMAKE_CXX_COMPILER_ID MATCHES Clang)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -ftemplate-backtrace-limit=0 -Wpedantic -Wextra -fcolor-diagnostics")

# MSVC Compiler
ELSEIF(CMAKE_CXX_COMPILER_ID MATCHES MSVC)
ELSE()
    MESSAGE(WARNING "You are using an unsupported compiler! Compilation has only been tested with Clang, GCC and MSVC.")
ENDIF()
