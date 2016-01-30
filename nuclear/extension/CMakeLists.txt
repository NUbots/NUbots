# Get the relative path to our extension directory
GET_FILENAME_COMPONENT(extension_include_path "${NUCLEAR_EXTENSION_DIR}/.." ABSOLUTE)
FILE(RELATIVE_PATH extension_include_path ${CMAKE_SOURCE_DIR} ${extension_include_path})

# Get our two include directories for extension
SET(extension_source_include_dir "${CMAKE_SOURCE_DIR}/${extension_include_path}")
SET(extension_binary_include_dir "${CMAKE_BINARY_DIR}/${extension_include_path}")

# Make our extension include directories variable
SET(NUCLEAR_EXTENSION_INCLUDE_DIRS
    ${extension_source_include_dir}
    ${extension_binary_include_dir}
    CACHE INTERNAL "Include directories for the extension folder and generated sources")
INCLUDE_DIRECTORIES(${NUCLEAR_EXTENSION_INCLUDE_DIRS})

# Use a recursive glob to get all c++ files in extension to use
FILE(GLOB_RECURSE src
        "${NUCLEAR_EXTENSION_DIR}/**.cpp"
        "${NUCLEAR_EXTENSION_DIR}/**.cc"
        "${NUCLEAR_EXTENSION_DIR}/**.ipp"
        "${NUCLEAR_EXTENSION_DIR}/**.hpp"
        "${NUCLEAR_EXTENSION_DIR}/**.c"
        "${NUCLEAR_EXTENSION_DIR}/**.h"
)

# Build a library from these files
ADD_LIBRARY(nuclear_extension extension.cpp ${src})

# Link our additional shared libraries
TARGET_LINK_LIBRARIES(nuclear_extension ${NUCLEAR_ADDITIONAL_SHARED_LIBRARIES})

# Put it in an IDE group for shared
SET_PROPERTY(TARGET nuclear_extension PROPERTY FOLDER "shared/")
