# If the user has provided their own CMakeLists.txt file in the extension dir use that otherwise we default to just
# building everything in there
if(EXISTS "${PROJECT_SOURCE_DIR}/${NUCLEAR_EXTENSION_DIR}/CMakeLists.txt")
  add_subdirectory("${PROJECT_SOURCE_DIR}/${NUCLEAR_EXTENSION_DIR}" "${PROJECT_BINARY_DIR}/${NUCLEAR_EXTENSION_DIR}")
else()
  include(NUClearExtension.cmake)
endif()
