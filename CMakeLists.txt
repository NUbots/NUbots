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
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")

# Include all of the globally shared link libraries
include(SharedLibraries)

# Include specific compiler options
include(Compiler)

# Set flag so NUClear uses a custom clock implementation
option(USE_CUSTOM_CLOCK "Replace NUClear's clock with a custom implementation." ON)
if(USE_CUSTOM_CLOCK)
  add_definitions(-DNUCLEAR_CUSTOM_CLOCK)
  add_subdirectory(shared/clock)
endif(USE_CUSTOM_CLOCK)

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

# Our banner file for placing at the top of the roles
set(NUCLEAR_ROLE_BANNER_FILE
    "${PROJECT_SOURCE_DIR}/cmake/banner.png"
    CACHE PATH "The path the banner to print at the start of each role execution" FORCE
)

if(BUILD_TESTS)
  enable_testing()
endif(BUILD_TESTS)

# Include the NUClear roles system
add_subdirectory(nuclear)

# Messages and utilities have a dependency on the custom clock implementation
if(USE_CUSTOM_CLOCK)
  target_link_libraries(${NUCLEAR_UTILITY_LIBRARIES} $<TARGET_OBJECTS:nubots_clock>)
  target_link_libraries(${NUCLEAR_MESSAGE_LIBRARIES} $<TARGET_OBJECTS:nubots_clock>)
endif(USE_CUSTOM_CLOCK)

# Enable sending binaries to robots
include(RobotSend)

if(BUILD_TESTS)
  add_subdirectory(shared/tests)
endif(BUILD_TESTS)
