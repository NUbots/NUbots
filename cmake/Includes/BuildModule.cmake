# Included in every module CMakeLists.txt
# We assume that module_name is defined by the including file

# Find all our files
FILE(GLOB_RECURSE src "src/**.cpp", "src/**.h")

# Include our own src dir and shared dir
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/shared)
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/shared)

# Add all our code to a library
ADD_LIBRARY(${module_name} ${src})
TARGET_LINK_LIBRARIES(${module_name} ${NUBOTS_SHARED_LIBRARIES})

# If we are doing tests then build the tests for this
IF(BUILD_TESTS)
    FILE(GLOB test_src "tests/**.cpp", "tests/**.h")
    ADD_EXECUTABLE(${module_name}Tests ${src} ${test_src})

    TARGET_LINK_LIBRARIES(${module_name}Tests
        ${NUBOTS_SHARED_LIBRARIES}
        ${JPGE_LIBRARIES}
        ${JSMN_LIBRARIES}
        ${NUCLEAR_LIBRARIES}
        ${ZMQ_LIBRARIES}
        ${PROTOBUF_LIBRARIES})
ENDIF()
