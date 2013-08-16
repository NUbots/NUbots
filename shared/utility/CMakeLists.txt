# This directory contains our message types and other shared types
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/shared/)

# This gets all of our source messages
FILE(GLOB_RECURSE src "*/*.cpp" "*/*.h")

# Build a library from these files
ADD_LIBRARY(utility ${src})
