# This gets all of our source files
file(GLOB_RECURSE test_src "*/**.cpp" "*/**.hpp")

if(test_src)
  add_executable(TestShared "${CMAKE_SOURCE_DIR}/${NUCLEAR_SHARED_DIR}/tests/test.cpp" ${test_src})
  target_link_libraries(TestShared PRIVATE nuclear::message nuclear::utility)

  set_target_properties(TestShared PROPERTIES FOLDER "shared/tests")

  # Add warnings. Use the default ROLE warnings setting if it's not configured manually
  if(NOT DEFINED NUCLEAR_UTILITY_WARNINGS)
    set(NUCLEAR_UTILITY_WARNINGS
        ${NUCLEAR_ROLE_WARNINGS}
        CACHE STRING "Compiler warnings used during utility test compilation"
    )
  endif()
  target_compile_options(TestShared PRIVATE ${NUCLEAR_UTILITY_WARNINGS})

  # Add the test
  add_test(
    NAME TestShared
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND ${CMAKE_CURRENT_BINARY_DIR}/TestShared
  )
endif()
