cmake_minimum_required(VERSION 3.0)
project(NUbots)

# ~~~
# This is needed so we can link in the custom clock object library to the message and utility libraries
# CMP0079: target_link_libraries() allows use with targets in other directories
# https://cmake.org/cmake/help/v3.16/policy/CMP0079.html
# ~~~
if(POLICY CMP0079)
  cmake_policy(SET CMP0079 NEW)
endif()

# Set on to prefer .a files over .so files for linking
option(STATIC_LIBRARIES
       "Prefer using static .a files rather than .so files. NOTE: This must be enabled from the start!" OFF
)

if(STATIC_LIBRARIES)
  set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_STATIC_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_SUFFIX})
endif()

# We use additional modules that cmake needs to know about
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/Modules/" "${PROJECT_SOURCE_DIR}/nuclear/cmake/Modules/")

# Default to do a debug build
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Debug' as none was specified.")
  set(CMAKE_BUILD_TYPE
      Debug
      CACHE STRING "Choose the type of build." FORCE
  )
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Default not to run the clang-tidy checks
option(ENABLE_CLANG_TIDY "Enable building with clang-tidy checks." OFF)
if(ENABLE_CLANG_TIDY)
  set(CMAKE_CXX_CLANG_TIDY clang-tidy)
  set(CMAKE_C_CLANG_TIDY clang-tidy)
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

# Make the compiler display colours always (even when we build with ninja)
if(CMAKE_CXX_COMPILER_ID MATCHES GNU)
  add_compile_options(-fdiagnostics-color=always)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES Clang)
  add_compile_options(-fcolor-diagnostics)
endif()

# Our banner file for placing at the top of the roles
set(NUCLEAR_ROLE_BANNER_FILE
    "${PROJECT_SOURCE_DIR}/cmake/banner.png"
    CACHE PATH "The path the banner to print at the start of each role execution" FORCE
)

# Include the NUClear roles system
add_subdirectory(nuclear)

# Setting to use the custom clock with an adaptable rate
option(USE_CUSTOM_CLOCK "Replace NUClear's clock with a custom implementation." ON)
if(USE_CUSTOM_CLOCK)
  add_subdirectory(shared/clock)
endif()

# Enable sending binaries to robots
include(RobotSend)

# Enable testing if we are building tests
if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(shared/tests)
endif()
