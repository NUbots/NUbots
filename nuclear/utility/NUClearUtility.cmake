# Get the path to our parent directory above the utility folder
get_filename_component(utility_parent_dir "${PROJECT_SOURCE_DIR}/${NUCLEAR_UTILITY_DIR}/.." ABSOLUTE)

# Grab all the utility code
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

add_library(nuclear_utility ${NUCLEAR_LINK_TYPE} "${CMAKE_CURRENT_SOURCE_DIR}/utility.cpp" ${src})

# Link in message library to utility
find_package(NUClear REQUIRED)
target_link_libraries(nuclear_utility PUBLIC NUClear::nuclear nuclear::message)
target_include_directories(nuclear_utility PUBLIC ${utility_parent_dir})

# If we have a libraries.cmake file then include it here
if(EXISTS "${PROJECT_SOURCE_DIR}/${NUCLEAR_UTILITY_DIR}/libraries.cmake")
  include("${PROJECT_SOURCE_DIR}/${NUCLEAR_UTILITY_DIR}/libraries.cmake")
endif()

# Generate in the lib folder so it gets installed
if(NUCLEAR_LINK_TYPE STREQUAL "SHARED")
  set_target_properties(nuclear_utility PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin/lib")
endif()

# Alias to the namespaced version
add_library(nuclear::utility ALIAS nuclear_utility)
