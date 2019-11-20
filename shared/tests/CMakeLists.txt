# This directory contains our message types and other shared types
include_directories(${CMAKE_SOURCE_DIR}/shared/)

include_directories(${NUCLEAR_MESSAGE_INCLUDE_DIRS})
include_directories(${NUCLEAR_UTILITY_INCLUDE_DIRS})
include_directories(${NUCLEAR_EXTENSION_INCLUDE_DIRS})

# This gets all of our source files
file(GLOB_RECURSE test_src "*/**.cpp" "*/**.h")

if(test_src)
  add_executable(TestShared "${CMAKE_SOURCE_DIR}/${NUCLEAR_SHARED_DIR}/tests/test.cpp" ${test_src})
  target_link_libraries(
    TestShared ${NUCLEAR_ADDITIONAL_SHARED_LIBRARIES} ${NUCLEAR_UTILITY_LIBRARIES} ${NUCLEAR_MESSAGE_LIBRARIES}
    ${NUClear_LIBRARIES} ${NUCLEAR_TEST_LIBRARIES}
  )

  set_property(TARGET TestShared PROPERTY FOLDER "shared/tests")

  # Add the test
  add_test(
    NAME TestShared
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND ${CMAKE_CURRENT_BINARY_DIR}/TestShared
  )
endif(test_src)
