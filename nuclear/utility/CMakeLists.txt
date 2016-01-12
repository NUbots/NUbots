# Get the relative path to our utility directory
FILE(RELATIVE_PATH utility_include_path ${CMAKE_SOURCE_DIR} "${NUCLEAR_UTILITY_DIR}/../")

# Make our message include directories variable
SET(NUCLEAR_UTILITY_INCLUDE_DIRS
    "${CMAKE_SOURCE_DIR}/${utility_include_path}"
    "${CMAKE_BINARY_DIR}/${utility_include_path}"
    CACHE INTERNAL "Include directories for the utility folder and generated sources")
INCLUDE_DIRECTORIES(${NUCLEAR_UTILITY_INCLUDE_DIRS})

# Use a recursive glob to get all c++ files in utility to use
FILE(GLOB_RECURSE src
        "${NUCLEAR_UTILITY_DIR}/**.cpp"
        "${NUCLEAR_UTILITY_DIR}/**.cc"
        "${NUCLEAR_UTILITY_DIR}/**.ipp"
        "${NUCLEAR_UTILITY_DIR}/**.hpp"
        "${NUCLEAR_UTILITY_DIR}/**.c"
        "${NUCLEAR_UTILITY_DIR}/**.h"
)

# Build a library from these files
ADD_LIBRARY(nuclear_utility utility.cpp ${src})

# Put it in an IDE group for shared
SET_PROPERTY(TARGET nuclear_utility PROPERTY FOLDER "shared/")
