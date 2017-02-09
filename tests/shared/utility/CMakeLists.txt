# This directory contains our message types and other shared types
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/shared/)

# This gets all of our source files
FILE(GLOB_RECURSE test_src "*/**.cpp" "*/**.h")

ADD_EXECUTABLE(test_utility "${CMAKE_SOURCE_DIR}/tests/test.cpp" ${test_src})

TARGET_LINK_LIBRARIES(test_utility ${NUBOTS_SHARED_LIBRARIES} ${LIBRARIES})

SET_PROPERTY(TARGET test_utility PROPERTY FOLDER "tests/shared/utility/")
