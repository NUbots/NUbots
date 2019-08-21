# If the user has provided their own CMakeLists.txt file in the messages dir use that otherwise we default to using the
# Neutron messaging system
if(EXISTS "${PROJECT_SOURCE_DIR}/${NUCLEAR_MESSAGE_DIR}/CMakeLists.txt")
  add_subdirectory("${PROJECT_SOURCE_DIR}/${NUCLEAR_MESSAGE_DIR}" "${PROJECT_BINARY_DIR}/${NUCLEAR_MESSAGE_DIR}")
else()
  include(Neutron.cmake)
endif()
