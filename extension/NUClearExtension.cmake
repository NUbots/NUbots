# Get the relative path to our extension directory
get_filename_component(extension_include_path "${NUCLEAR_EXTENSION_DIR}/.." ABSOLUTE)
file(RELATIVE_PATH extension_include_path ${PROJECT_SOURCE_DIR} ${extension_include_path})

# Get our two include directories for extension
set(extension_source_include_dir "${PROJECT_SOURCE_DIR}/${extension_include_path}")
set(extension_binary_include_dir "${PROJECT_BINARY_DIR}/${extension_include_path}")

# Make our extension include directories variable
set(
  NUCLEAR_EXTENSION_INCLUDE_DIRS
  ${extension_source_include_dir} ${extension_binary_include_dir}
  CACHE INTERNAL "Include directories for the extension folder and generated sources"
)

# Include both our include directory message and utility (since we can use them)
include_directories(${NUCLEAR_EXTENSION_INCLUDE_DIRS})
include_directories(${NUCLEAR_UTILITY_INCLUDE_DIRS})
include_directories(${NUCLEAR_MESSAGE_INCLUDE_DIRS})

# Get the relative path to our extension directory
get_filename_component(extension_path "${NUCLEAR_EXTENSION_DIR}" ABSOLUTE)
file(RELATIVE_PATH extension_rel_path ${PROJECT_SOURCE_DIR} ${extension_path})

# Clear our list of NUClear extension libraries
unset(NUCLEAR_EXTENSION_LIBRARIES CACHE)

# If the user has a CMakeLists in the extension directory use that This CMakeLists must set the cache variable
# NUCLEAR_EXTENSION_LIBRARIES itself
if(EXISTS "${PROJECT_SOURCE_DIR}/${NUCLEAR_EXTENSION_DIR}/CMakeLists.txt")
  add_subdirectory("${PROJECT_SOURCE_DIR}/${NUCLEAR_EXTENSION_DIR}" "${PROJECT_BINARY_DIR}/${NUCLEAR_EXTENSION_DIR}")

  # Otherwise we assume we build them ourself
else()
  # Using all extensions
  file(
    GLOB_RECURSE
    src
    "${PROJECT_SOURCE_DIR}/${NUCLEAR_EXTENSION_DIR}/**.cpp"
    "${PROJECT_SOURCE_DIR}/${NUCLEAR_EXTENSION_DIR}/**.cc"
    "${PROJECT_SOURCE_DIR}/${NUCLEAR_EXTENSION_DIR}/**.c"
    "${PROJECT_SOURCE_DIR}/${NUCLEAR_EXTENSION_DIR}/**.ipp"
    "${PROJECT_SOURCE_DIR}/${NUCLEAR_EXTENSION_DIR}/**.hpp"
    "${PROJECT_SOURCE_DIR}/${NUCLEAR_EXTENSION_DIR}/**.hh"
    "${PROJECT_SOURCE_DIR}/${NUCLEAR_EXTENSION_DIR}/**.h"
  )

  # If we had source files build them into a library
  if(src)
    # Build a library from these files
    add_library(nuclear_extension extension.cpp ${src})

    # Link our additional shared libraries
    target_link_libraries(nuclear_extension ${NUCLEAR_ADDITIONAL_SHARED_LIBRARIES} ${NUCLEAR_UTILITY_LIBRARIES}
                          ${NUCLEAR_MESSAGE_LIBRARIES})

    # Add to our list of NUClear extension libraries
    set(
      NUCLEAR_EXTENSION_LIBRARIES
      nuclear_extension
      CACHE INTERNAL "List of libraries that are built as extensions" FORCE
    )

    # Put it in an IDE group for shared
    set_property(TARGET nuclear_extension PROPERTY FOLDER "shared/")
  endif()
endif()
