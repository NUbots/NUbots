# Get the relative path to our extension directory
GET_FILENAME_COMPONENT(extension_include_path "${NUCLEAR_EXTENSION_DIR}/.." ABSOLUTE)
FILE(RELATIVE_PATH extension_include_path ${PROJECT_SOURCE_DIR} ${extension_include_path})

# Get our two include directories for extension
SET(extension_source_include_dir "${PROJECT_SOURCE_DIR}/${extension_include_path}")
SET(extension_binary_include_dir "${PROJECT_BINARY_DIR}/${extension_include_path}")

# Make our extension include directories variable
SET(NUCLEAR_EXTENSION_INCLUDE_DIRS
    ${extension_source_include_dir}
    ${extension_binary_include_dir}
    CACHE INTERNAL "Include directories for the extension folder and generated sources")

# Include both our include directory message and utility (since we can use them)
INCLUDE_DIRECTORIES(${NUCLEAR_EXTENSION_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${NUCLEAR_UTILITY_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${NUCLEAR_MESSAGE_INCLUDE_DIRS})

# Get the relative path to our extension directory
GET_FILENAME_COMPONENT(extension_path "${NUCLEAR_EXTENSION_DIR}" ABSOLUTE)
FILE(RELATIVE_PATH extension_rel_path ${PROJECT_SOURCE_DIR} ${extension_path})

# Clear our list of NUClear extension libraries
UNSET(NUCLEAR_EXTENSION_LIBRARIES CACHE)

# If the user has a CMakeLists in the extension directory use that
# This CMakeLists must set the cache variable NUCLEAR_EXTENSION_LIBRARIES itself
IF(EXISTS "${PROJECT_SOURCE_DIR}/${NUCLEAR_EXTENSION_DIR}/CMakeLists.txt")
    ADD_SUBDIRECTORY("${PROJECT_SOURCE_DIR}/${NUCLEAR_EXTENSION_DIR}" "${PROJECT_BINARY_DIR}/${NUCLEAR_EXTENSION_DIR}")

# Otherwise we assume we build them ourself
ELSE()
    # Using all extensions
    FILE(GLOB_RECURSE src
            "${PROJECT_SOURCE_DIR}/${NUCLEAR_EXTENSION_DIR}/**.cpp"
            "${PROJECT_SOURCE_DIR}/${NUCLEAR_EXTENSION_DIR}/**.cc"
            "${PROJECT_SOURCE_DIR}/${NUCLEAR_EXTENSION_DIR}/**.ipp"
            "${PROJECT_SOURCE_DIR}/${NUCLEAR_EXTENSION_DIR}/**.hpp"
            "${PROJECT_SOURCE_DIR}/${NUCLEAR_EXTENSION_DIR}/**.c"
            "${PROJECT_SOURCE_DIR}/${NUCLEAR_EXTENSION_DIR}/**.h"
    )

    # If we had source files build them into a library
    IF(src)
        # Build a library from these files
        ADD_LIBRARY(nuclear_extension extension.cpp ${src})

        # Link our additional shared libraries
        TARGET_LINK_LIBRARIES(nuclear_extension ${NUCLEAR_ADDITIONAL_SHARED_LIBRARIES} ${NUCLEAR_UTILITY_LIBRARIES} ${NUCLEAR_MESSAGE_LIBRARIES})

        # Add to our list of NUClear extension libraries
        SET(NUCLEAR_EXTENSION_LIBRARIES nuclear_extension CACHE INTERNAL "List of libraries that are built as extensions" FORCE)

        # Put it in an IDE group for shared
        SET_PROPERTY(TARGET nuclear_extension PROPERTY FOLDER "shared/")
    ENDIF()
ENDIF()

