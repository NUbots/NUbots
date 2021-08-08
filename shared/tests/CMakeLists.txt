# This gets all of our source files
file(GLOB_RECURSE test_src "*/**.cpp" "*/**.hpp")

if(test_src)
  add_executable(testshared "${CMAKE_SOURCE_DIR}/${NUCLEAR_SHARED_DIR}/tests/test.cpp" ${test_src})
  target_link_libraries(testshared PRIVATE nuclear::message nuclear::utility)

  set_target_properties(testshared PROPERTIES FOLDER "shared/tests")

  # Add the test
  add_test(
    NAME testshared
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND ${CMAKE_CURRENT_BINARY_DIR}/testshared
  )
endif()
