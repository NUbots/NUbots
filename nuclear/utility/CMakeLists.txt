# If the user has provided their own CMakeLists.txt file in the utility dir use that
# otherwise we default to just building everything in there
IF(EXISTS "${PROJECT_SOURCE_DIR}/${NUCLEAR_UTILITY_DIR}/CMakeLists.txt")
    ADD_SUBDIRECTORY("${PROJECT_SOURCE_DIR}/${NUCLEAR_UTILITY_DIR}" "${PROJECT_BINARY_DIR}/${NUCLEAR_UTILITY_DIR}")
ELSE()
    INCLUDE(NUClearUtility.cmake)
ENDIF()
