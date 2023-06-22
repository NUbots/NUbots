# Get the path to our parent directory above the extension folder
get_filename_component(extension_parent_dir "${PROJECT_SOURCE_DIR}/${NUCLEAR_UTILITY_DIR}/.." ABSOLUTE)

# Grab all the extension code
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

add_library(nuclear_extension ${NUCLEAR_LINK_TYPE} "${CMAKE_CURRENT_SOURCE_DIR}/extension.cpp" ${src})

# Link in message and utility
target_link_libraries(nuclear_extension PUBLIC nuclear::message nuclear::utility)
target_include_directories(nuclear_extension PUBLIC ${extension_parent_dir})

# If we have a libraries.cmake file then include it here
if(EXISTS "${PROJECT_SOURCE_DIR}/${NUCLEAR_EXTENSION_DIR}/libraries.cmake")
  include("${PROJECT_SOURCE_DIR}/${NUCLEAR_EXTENSION_DIR}/libraries.cmake")
endif()

# Generate in the lib folder so it gets installed
if(NUCLEAR_LINK_TYPE STREQUAL "SHARED")
  set_property(TARGET nuclear_extension PROPERTY LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin/lib")
endif()

# Alias to the namespaced version
add_library(nuclear::extension ALIAS nuclear_extension)
