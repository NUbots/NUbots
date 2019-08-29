cmake_minimum_required(VERSION 2.8)
# This directory contains our message types and other shared types
include_directories(${CMAKE_SOURCE_DIR}/shared/ ${CMAKE_SOURCE_DIR}/ SYSTEM)
include_directories(${NUCLEAR_MESSAGE_INCLUDE_DIRS})

# We need Eigen3
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${Eigen3_INCLUDE_DIRS})

# Required for vision utility
find_package(Aravis REQUIRED)
find_package(glib2 REQUIRED)
include_directories(SYSTEM ${Aravis_INCLUDE_DIRS})
include_directories(SYSTEM ${glib2_INCLUDE_DIRS})

# Set project name found variable
string(TOUPPER ${CMAKE_PROJECT_NAME} PROJECT_NAME)
set(${PROJECT_NAME}_PROJECT_FOUND TRUE)

# Include tools
include(ConfigureUtilities.cmake)

# Note where we are
set(NUTILITIES_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Find all subdirectories
subdirlist(directories ${CMAKE_CURRENT_SOURCE_DIR})

# initialise variables which will contain our collected files and libs
unset(NUTILITIES_LIBRARIES_FILES CACHE)

# Add each subdirectory with selection for loaded libraries
foreach(dir ${directories})
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/${dir})
endforeach()

# Compile the nutilities library
add_library(nutilities ${NUTILITIES_LIBRARIES_FILES})

# Link in our messaging library to ensure they are built first Also link to our shared libraries
target_link_libraries(nutilities ${NUCLEAR_MESSAGE_LIBRARIES})
target_link_libraries(nutilities ${NUCLEAR_ADDITIONAL_SHARED_LIBRARIES})

# SET(NUCLEAR_UTILITY_LIBRARIES ${NUCLEAR_UTILITY_LIBRARIES} ${NUTILITIES_LIBRARIES} CACHE INTERNAL "Library for NUClear
# utilities" FORCE)
set(NUCLEAR_UTILITY_LIBRARIES nutilities CACHE INTERNAL "Library for NUClear utilities" FORCE)
