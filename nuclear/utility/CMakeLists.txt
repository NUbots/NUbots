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

# Get the relative path to our utility directory
GET_FILENAME_COMPONENT(utility_path "${NUCLEAR_UTILITY_DIR}" ABSOLUTE)
FILE(RELATIVE_PATH utility_rel_path ${CMAKE_SOURCE_DIR} ${utility_path})

#Set bin directory to mirror source directory
UNSET(NUCLEAR_UTILITY_LIBRARIES CACHE)

#Get utilities library
ADD_SUBDIRECTORY(${NUCLEAR_UTILITY_DIR} "${CMAKE_BINARY_DIR}/${utility_rel_path}")

# Build a library from these files
ADD_LIBRARY(nuclear_utility utility.cpp )

# Link our additional shared libraries
TARGET_LINK_LIBRARIES(nuclear_utility ${NUCLEAR_ADDITIONAL_SHARED_LIBRARIES} ${NUCLEAR_UTILITY_LIBRARIES})

# Put it in an IDE group for shared
SET_PROPERTY(TARGET nuclear_utility PROPERTY FOLDER "shared/")
