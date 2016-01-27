# Get the relative path to our utility directory
GET_FILENAME_COMPONENT(utility_include_path "${NUCLEAR_UTILITY_DIR}/.." ABSOLUTE)
FILE(RELATIVE_PATH utility_include_path ${CMAKE_SOURCE_DIR} ${utility_include_path})

# Get our two include directories for utilty
SET(utility_source_include_dir "${CMAKE_SOURCE_DIR}/${utility_include_path}")
SET(utility_binary_include_dir "${CMAKE_BINARY_DIR}/${utility_include_path}")

# Make our utility include directories variable
SET(NUCLEAR_UTILITY_INCLUDE_DIRS
    ${utility_source_include_dir}
    ${utility_binary_include_dir}
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

# Link our additional shared libraries
TARGET_LINK_LIBRARIES(nuclear_utility ${NUCLEAR_ADDITIONAL_SHARED_LIBRARIES})

# Put it in an IDE group for shared
SET_PROPERTY(TARGET nuclear_utility PROPERTY FOLDER "shared/")
