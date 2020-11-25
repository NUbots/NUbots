# This gets all of our source files
file(GLOB_RECURSE test_src "*/**.cpp" "*/**.h")

if(test_src)
  add_executable(TestShared "${CMAKE_SOURCE_DIR}/${NUCLEAR_SHARED_DIR}/tests/test.cpp" ${test_src})
  target_link_libraries(TestShared nuclear::message nuclear::utility)

  set_property(TARGET TestShared PROPERTY FOLDER "shared/tests")

  # Add the test
  add_test(
    NAME TestShared
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND ${CMAKE_CURRENT_BINARY_DIR}/TestShared
  )
endif(test_src)
