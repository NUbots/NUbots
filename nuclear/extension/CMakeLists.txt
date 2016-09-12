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

# Get the relative path to our extension directory
GET_FILENAME_COMPONENT(extension_path "${NUCLEAR_EXTENSION_DIR}" ABSOLUTE)
FILE(RELATIVE_PATH extension_rel_path ${CMAKE_SOURCE_DIR} ${extension_path})

# Clear our list of NUClear extension libraries
UNSET(NUCLEAR_EXTENSION_LIBRARIES CACHE)

# If the user has a CMakeLists in the extension directory use that
# This CMakeLists must set the cache variable NUCLEAR_EXTENSION_LIBRARIES itself
IF(EXISTS "${NUCLEAR_EXTENSION_DIR}/CMakeLists.txt")
    ADD_SUBDIRECTORY(${NUCLEAR_EXTENSION_DIR} "${CMAKE_BINARY_DIR}/${extension_rel_path}")

# Otherwise we assume we build them ourself
ELSE()
    # Using all extensions
    FILE(GLOB_RECURSE src
            "${NUCLEAR_EXTENSION_DIR}/**.cpp"
            "${NUCLEAR_EXTENSION_DIR}/**.cc"
            "${NUCLEAR_EXTENSION_DIR}/**.ipp"
            "${NUCLEAR_EXTENSION_DIR}/**.hpp"
            "${NUCLEAR_EXTENSION_DIR}/**.c"
            "${NUCLEAR_EXTENSION_DIR}/**.h"
    )

    # If we had source files build them into a library
    IF(src)
        # Build a library from these files
        ADD_LIBRARY(nuclear_extension extension.cpp ${src})

        # Link our additional shared libraries
        TARGET_LINK_LIBRARIES(nuclear_extension ${NUCLEAR_ADDITIONAL_SHARED_LIBRARIES})

        # Add to our list of NUClear extension libraries
        SET(NUCLEAR_EXTENSION_LIBRARIES nuclear_extension CACHE INTERNAL "List of libraries that are built as extensions" FORCE)

        # Put it in an IDE group for shared
        SET_PROPERTY(TARGET nuclear_extension PROPERTY FOLDER "shared/")
    ENDIF()
ENDIF()

