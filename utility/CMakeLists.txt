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

# Clear our list of NUClear utility libraries
UNSET(NUCLEAR_UTILITY_LIBRARIES CACHE)

# If the user has a CMakeLists in the utility directory use that
# This CMakeLists must set the cache variable NUCLEAR_UTILITY_LIBRARIES itself
IF(EXISTS "${NUCLEAR_UTILITY_DIR}/CMakeLists.txt")
    ADD_SUBDIRECTORY(${NUCLEAR_UTILITY_DIR} "${CMAKE_BINARY_DIR}/${utility_rel_path}")

# Otherwise we assume we build them ourself
ELSE()
    # Using all utilities
    FILE(GLOB_RECURSE src
            "${NUCLEAR_UTILITY_DIR}/**.cpp"
            "${NUCLEAR_UTILITY_DIR}/**.cc"
            "${NUCLEAR_UTILITY_DIR}/**.ipp"
            "${NUCLEAR_UTILITY_DIR}/**.hpp"
            "${NUCLEAR_UTILITY_DIR}/**.c"
            "${NUCLEAR_UTILITY_DIR}/**.h"
    )

    # If we had source files build them into a library
    IF(src)
        # Build a library from these files
        ADD_LIBRARY(nuclear_utility utility.cpp ${src})

        # Link our additional shared libraries
        TARGET_LINK_LIBRARIES(nuclear_utility ${NUCLEAR_ADDITIONAL_SHARED_LIBRARIES})

        # Add to our list of NUClear utility libraries
        SET(NUCLEAR_UTILITY_LIBRARIES nuclear_utility CACHE INTERNAL "List of libraries that are built as utilities" FORCE)

        # Put it in an IDE group for shared
        SET_PROPERTY(TARGET nuclear_utility PROPERTY FOLDER "shared/")
    ENDIF()
ENDIF()

