# Silence warnings about target_sources converting relative paths to abolutes
cmake_policy(SET CMP0076 NEW)

add_executable(TestNUClearRoles "${CMAKE_CURRENT_SOURCE_DIR}/test.cpp")

# Add all of the tests
add_subdirectory(message)
target_link_libraries(TestNUClearRoles PUBLIC nuclear::message)

set_property(TARGET TestNUClearRoles PROPERTY FOLDER "nuclear/tests")

# Add the test
add_test(
  NAME TestNUClearRoles
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/TestNUClearRoles
)
