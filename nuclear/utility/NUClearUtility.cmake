# Get the relative path to our utility directory
get_filename_component(utility_include_path "${NUCLEAR_UTILITY_DIR}/.." ABSOLUTE)
file(RELATIVE_PATH utility_include_path ${PROJECT_SOURCE_DIR} ${utility_include_path})

# Get our two include directories for utility
set(utility_source_include_dir "${PROJECT_SOURCE_DIR}/${utility_include_path}")
set(utility_binary_include_dir "${PROJECT_BINARY_DIR}/${utility_include_path}")

# Make our utility include directories variable
set(NUCLEAR_UTILITY_INCLUDE_DIRS
    ${utility_source_include_dir} ${utility_binary_include_dir}
    CACHE INTERNAL "Include directories for the utility folder and generated sources"
)

# Include both our include directory and messages (since we can use them)
include_directories(${NUCLEAR_UTILITY_INCLUDE_DIRS})
include_directories(${NUCLEAR_MESSAGE_INCLUDE_DIRS})

# Get the relative path to our utility directory
get_filename_component(utility_path "${NUCLEAR_UTILITY_DIR}" ABSOLUTE)
file(RELATIVE_PATH utility_rel_path ${PROJECT_SOURCE_DIR} ${utility_path})

# Clear our list of NUClear utility libraries
unset(NUCLEAR_UTILITY_LIBRARIES CACHE)

# Using all utilities
file(
  GLOB_RECURSE
  src
  "${PROJECT_SOURCE_DIR}/${NUCLEAR_UTILITY_DIR}/**.cpp"
  "${PROJECT_SOURCE_DIR}/${NUCLEAR_UTILITY_DIR}/**.cc"
  "${PROJECT_SOURCE_DIR}/${NUCLEAR_UTILITY_DIR}/**.c"
  "${PROJECT_SOURCE_DIR}/${NUCLEAR_UTILITY_DIR}/**.ipp"
  "${PROJECT_SOURCE_DIR}/${NUCLEAR_UTILITY_DIR}/**.hpp"
  "${PROJECT_SOURCE_DIR}/${NUCLEAR_UTILITY_DIR}/**.hh"
  "${PROJECT_SOURCE_DIR}/${NUCLEAR_UTILITY_DIR}/**.h"
)

# If we had source files build them into a library
if(src)
  # Build a library from these files
  add_library(nuclear_utility utility.cpp ${src})

  # Link our additional shared libraries
  target_link_libraries(nuclear_utility ${NUCLEAR_ADDITIONAL_SHARED_LIBRARIES} ${NUCLEAR_MESSAGE_LIBRARIES})

  # Add to our list of NUClear utility libraries
  set(NUCLEAR_UTILITY_LIBRARIES
      nuclear_utility
      CACHE INTERNAL "List of libraries that are built as utilities" FORCE
  )

  # Put it in an IDE group for shared
  set_property(TARGET nuclear_utility PROPERTY FOLDER "shared/")
endif()
