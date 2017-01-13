# If the user has provided their own CMakeLists.txt file in the messages dir use that
# otherwise we default to using the Neutron messaging system
IF(EXISTS "${PROJECT_SOURCE_DIR}/${NUCLEAR_MESSAGE_DIR}/CMakeLists.txt")
    ADD_SUBDIRECTORY("${PROJECT_SOURCE_DIR}/${NUCLEAR_MESSAGE_DIR}" "${PROJECT_BINARY_DIR}/${NUCLEAR_MESSAGE_DIR}")
ELSE()
    INCLUDE(Neutron.cmake)
ENDIF()
