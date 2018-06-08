# Get the relative path to our utility directory
GET_FILENAME_COMPONENT(utility_include_path "${NUCLEAR_UTILITY_DIR}/.." ABSOLUTE)
FILE(RELATIVE_PATH utility_include_path ${PROJECT_SOURCE_DIR} ${utility_include_path})

# Get our two include directories for utility
SET(utility_source_include_dir "${PROJECT_SOURCE_DIR}/${utility_include_path}")
SET(utility_binary_include_dir "${PROJECT_BINARY_DIR}/${utility_include_path}")

# Make our utility include directories variable
SET(NUCLEAR_UTILITY_INCLUDE_DIRS
    ${utility_source_include_dir}
    ${utility_binary_include_dir}
    CACHE INTERNAL "Include directories for the utility folder and generated sources")

# Include both our include directory and messages (since we can use them)
INCLUDE_DIRECTORIES(${NUCLEAR_UTILITY_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${NUCLEAR_MESSAGE_INCLUDE_DIRS})

# Get the relative path to our utility directory
GET_FILENAME_COMPONENT(utility_path "${NUCLEAR_UTILITY_DIR}" ABSOLUTE)
FILE(RELATIVE_PATH utility_rel_path ${PROJECT_SOURCE_DIR} ${utility_path})

# Clear our list of NUClear utility libraries
UNSET(NUCLEAR_UTILITY_LIBRARIES CACHE)

# Using all utilities
FILE(GLOB_RECURSE src
        "${PROJECT_SOURCE_DIR}/${NUCLEAR_UTILITY_DIR}/**.cpp"
        "${PROJECT_SOURCE_DIR}/${NUCLEAR_UTILITY_DIR}/**.cc"
        "${PROJECT_SOURCE_DIR}/${NUCLEAR_UTILITY_DIR}/**.c"
        "${PROJECT_SOURCE_DIR}/${NUCLEAR_UTILITY_DIR}/**.ipp"
        "${PROJECT_SOURCE_DIR}/${NUCLEAR_UTILITY_DIR}/**.hpp"
        "${PROJECT_SOURCE_DIR}/${NUCLEAR_UTILITY_DIR}/**.hh"
        "${PROJECT_SOURCE_DIR}/${NUCLEAR_UTILITY_DIR}/**.h"
)

# If we had source files build them into a library
IF(src)
    # Build a library from these files
    ADD_LIBRARY(nuclear_utility utility.cpp ${src})

    # Link our additional shared libraries
    TARGET_LINK_LIBRARIES(nuclear_utility ${NUCLEAR_ADDITIONAL_SHARED_LIBRARIES} ${NUCLEAR_MESSAGE_LIBRARIES})

    # Add to our list of NUClear utility libraries
    SET(NUCLEAR_UTILITY_LIBRARIES nuclear_utility CACHE INTERNAL "List of libraries that are built as utilities" FORCE)

    # Put it in an IDE group for shared
    SET_PROPERTY(TARGET nuclear_utility PROPERTY FOLDER "shared/")
ENDIF()

