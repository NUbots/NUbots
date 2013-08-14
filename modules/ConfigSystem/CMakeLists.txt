# Set our module name so that the rest of this file can be generated
SET(module_name "ConfigSystem")

# YOU SHOULD NOT HAVE TO TOUCH ANYTHING BELOW THIS LINE

# Find all our files
FILE(GLOB src "src/*.cpp", "src/*.h")

# Include our own src dir and the shared dirs
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/shared)
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/shared)

# Add all our code to a library
ADD_LIBRARY(${module_name} ${src})
TARGET_LINK_LIBRARIES(${module_name} messages)

# If we are doing tests, then build the test for this
IF(BUILD_TESTS)
    FILE(GLOB test_src "tests/**.cpp", "tests/**.h")
    ADD_EXECUTABLE(${module_name}Tests ${src} ${test_src})
    
    TARGET_LINK_LIBRARIES(${module_name}Tests
        messages
        ${NUCLEAR_LIBRARIES}
        ${ZMQ_LIBRARIES}
        ${PROTOBUF_LIBRARIES})
ENDIF()