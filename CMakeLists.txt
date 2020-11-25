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
endif(ENABLE_CLANG_TIDY)

# Enable address sanitizer
option(USE_ASAN "Enable address sanitization" OFF)
if(USE_ASAN)
  add_compile_options(-fsanitize=address -fno-omit-frame-pointer -U_FORTIFY_SOURCE -fno-common)
  add_link_options(-fsanitize=address)
  link_libraries(asan)
endif(USE_ASAN)

# Enable testing if we are building tests
if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(shared/tests)
endif(BUILD_TESTS)

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

# Set flag so NUClear uses a custom clock implementation
option(USE_CUSTOM_CLOCK "Replace NUClear's clock with a custom implementation." ON)
if(USE_CUSTOM_CLOCK)
  add_definitions(-DNUCLEAR_CUSTOM_CLOCK)
  add_subdirectory(shared/clock)
endif(USE_CUSTOM_CLOCK)

# Our banner file for placing at the top of the roles
set(NUCLEAR_ROLE_BANNER_FILE
"${PROJECT_SOURCE_DIR}/cmake/banner.png"
CACHE PATH "The path the banner to print at the start of each role execution" FORCE
)

# Include the NUClear roles system
add_subdirectory(nuclear)

# Some libraries seem to not know when to make things advanced so we give them a hand here
mark_as_advanced(SQLiteCpp_DIR fmt_DIR yaml-cpp_DIR NUClear_DIR Eigen3_DIR)

# Hide some cmake variables that will never be used
mark_as_advanced(
  CMAKE_INSTALL_PREFIX
  CURSES_CURSES_H_PATH
  CURSES_FORM_LIBRARY
  CURSES_HAVE_CURSES_H
  OMPLAPP_LIBRARY
  OMPL_CONFIG
  OMPL_DIR
  OMPL_INCLUDE_DIRS
  OMPL_LIBRARIES
  OMPL_LIBRARY
)

# Messages and utilities have a dependency on the custom clock implementation
if(USE_CUSTOM_CLOCK)
  target_link_libraries(${NUCLEAR_UTILITY_LIBRARIES} $<TARGET_OBJECTS:nubots_clock>)
  target_link_libraries(${NUCLEAR_MESSAGE_LIBRARIES} $<TARGET_OBJECTS:nubots_clock>)
endif(USE_CUSTOM_CLOCK)

# Enable sending binaries to robots
include(RobotSend)
