# This directory contains our message types and other shared types
include_directories(${CMAKE_SOURCE_DIR}/shared/)

# This gets all of our source files
file(GLOB_RECURSE test_src "*/**.cpp" "*/**.h")

add_executable(test_utility "${CMAKE_SOURCE_DIR}/tests/test.cpp" ${test_src})

target_link_libraries(test_utility ${NUBOTS_SHARED_LIBRARIES} ${LIBRARIES})

set_property(TARGET test_utility PROPERTY FOLDER "tests/shared/utility/")
