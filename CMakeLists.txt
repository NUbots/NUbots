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
cmake_minimum_required(VERSION 3.20)
project(NUbots)

# The GENERATED source file property is now visible in all directories
cmake_policy(SET CMP0118 NEW)

# ~~~
# This is needed so we can link in the custom clock object library to the message and utility libraries
# CMP0079: target_link_libraries() allows use with targets in other directories
# https://cmake.org/cmake/help/v3.16/policy/CMP0079.html
# ~~~
if(POLICY CMP0079)
  cmake_policy(SET CMP0079 NEW)
endif()

# Enable testing if we are building tests
if(BUILD_TESTS)
  enable_testing()
endif()

# We use additional modules that cmake needs to know about
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/Modules/" "${PROJECT_SOURCE_DIR}/nuclear/cmake/Modules/")

# If this option is set we are building using continous integration
option(CI_BUILD "Enable build options for building in the CI server" OFF)

# TODO(DevOpsTeam): Remove this once we have fixed warnings (including clang-tidy warnings) codebase wide to add
# clang-tidy to the builds and prevent new warnings being introduced
set(CI_BUILD
    OFF
    CACHE
      BOOL
      "This disables the effects of the CI_BUILD variable. We should remove this set() once warnings have been fixed."
)

# Default to do a release build
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE
      Release
      CACHE STRING "Choose the type of build." FORCE
  )
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Allow selection of subcontroller for conditional build
if(NOT SUBCONTROLLER)
  message(STATUS "Setting subcontroller to 'OpenCR' as none was specified.")
  set(SUBCONTROLLER
      OpenCR
      CACHE STRING "Choose the subcontroller to build for." FORCE
  )
  # Set the possible values of subcontroller for cmake-gui
  set_property(CACHE SUBCONTROLLER PROPERTY STRINGS "OpenCR")
endif()

# Default not to run the clang-tidy checks, default to whatever our CI_BUILD is
option(ENABLE_CLANG_TIDY "Enable building with clang-tidy checks." OFF)
if(ENABLE_CLANG_TIDY) # Turn back on for CI_BUILD once we have fixed all the warnings
  find_package(PythonInterp 3 REQUIRED)
  set(CMAKE_CXX_CLANG_TIDY "${PYTHON_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/cmake/scripts/clang-tidy.py"
                           "${PROJECT_BINARY_DIR}/clang-tidy-fixes" clang-tidy
  )
  set(CMAKE_C_CLANG_TIDY ${CMAKE_CXX_CLANG_TIDY})

  # Create a target that will apply clang-tidy fixes to the codebase
  add_custom_target(
    apply-clang-tidy
    COMMAND clang-apply-replacements --format --style=file --style-config="${PROJECT_SOURCE_DIR}"
            --remove-change-desc-files "${PROJECT_BINARY_DIR}/clang-tidy-fixes"
    COMMENT "Applying fixes from clang-tidy to the codebase."
  )
endif()

# If we are doing a CI build then we want to enable -Werror when compiling warnings are bad. We will also make it fail
# if clang-tidy has an error
if(CI_BUILD)
  add_compile_options(-Werror)
  # Turn these back on once we have fixed all the clang-tidy warnings * set(CMAKE_CXX_CLANG_TIDY ${CMAKE_CXX_CLANG_TIDY}
  # -warnings-as-errors=*) * set(CMAKE_C_CLANG_TIDY ${CMAKE_C_CLANG_TIDY} -warnings-as-errors=*)
endif()

# Enable address sanitizer
option(USE_ASAN "Enable address sanitization" OFF)
if(USE_ASAN)
  add_compile_options(-fsanitize=address -fno-omit-frame-pointer -U_FORTIFY_SOURCE -fno-common)
  add_link_options(-fsanitize=address)
  link_libraries(asan)
endif()

# Output the compilation database
set(CMAKE_EXPORT_COMPILE_COMMANDS
    ON
    CACHE STRING "Enable/Disable output of compile commands during generation." FORCE
)

# Enable all warnings that we can
add_compile_options(-Wall -Wpedantic -Wextra)

# This is needed everywhere so that we can catch segmentation faults
add_compile_options(-fnon-call-exceptions)

# We use C++20
add_compile_options(-std=c++20)

# Make NUClear use the system clock, as we are syncing it with ptp
add_compile_definitions(NUCLEAR_CLOCK_TYPE=std::chrono::steady_clock)

# Make the compiler display colours always (even when we build with ninja)
if(CMAKE_CXX_COMPILER_ID MATCHES GNU)
  add_compile_options(-fdiagnostics-color=always)
elseif(CMAKE_CXX_COMPILER_ID MATCHES Clang)
  add_compile_options(-fcolor-diagnostics)
endif()

# Our banner file for placing at the top of the roles
set(NUCLEAR_ROLE_BANNER_FILE
    "${PROJECT_SOURCE_DIR}/cmake/banner.png"
    CACHE PATH "The path the banner to print at the start of each role execution" FORCE
)

# Build **ALL** modules, even those which aren't otherwise in a role
option(BUILD_ALL_MODULES "Create a role with all modules included" OFF)

# Include the NUClear roles system
add_subdirectory(nuclear)

# Add the shared tests. Must happen after adding NUClear so that this variable is cached
if(BUILD_TESTS)
  add_subdirectory(${NUCLEAR_SHARED_DIR}/tests)
endif()

# Generate VSCode launch configurations
find_package(PythonInterp 3 REQUIRED)
list(JOIN NUCLEAR_ROLES " " launch_roles)
add_custom_command(
  OUTPUT "${PROJECT_SOURCE_DIR}/.vscode/launch.json"
  COMMAND
    ${PYTHON_EXECUTABLE} ARGS "${PROJECT_SOURCE_DIR}/cmake/scripts/generate_launch_configurations.py" "-s"
    "${PROJECT_SOURCE_DIR}" "-b" "${PROJECT_BINARY_DIR}" "-l" "${PROJECT_SOURCE_DIR}/.vscode/launch.json" "-a"
    "${USE_ASAN}" "${launch_roles}"
  DEPENDS "${PROJECT_SOURCE_DIR}/cmake/scripts/generate_launch_configurations.py"
  USES_TERMINAL
  COMMENT "Generating VSCode launch configurations"
)
add_custom_target(generate_launch_configurations ALL DEPENDS "${PROJECT_SOURCE_DIR}/.vscode/launch.json")
