# If the user has provided their own CMakeLists.txt file in the extension dir use that
# otherwise we default to just building everything in there
IF(EXISTS "${PROJECT_SOURCE_DIR}/${NUCLEAR_EXTENSION_DIR}/CMakeLists.txt")
    ADD_SUBDIRECTORY("${PROJECT_SOURCE_DIR}/${NUCLEAR_EXTENSION_DIR}" "${PROJECT_BINARY_DIR}/${NUCLEAR_EXTENSION_DIR}")
ELSE()
    INCLUDE(NUClearExtension.cmake)
ENDIF()
