# Silence warnings about target_sources converting relative paths to abolutes
cmake_policy(SET CMP0076 NEW)

find_package(Catch2 REQUIRED)

add_executable(TestNUClearRoles)

# Add all of the tests
add_subdirectory(message)
target_link_libraries(TestNUClearRoles PUBLIC nuclear::message)
target_link_libraries(TestNUClearRoles PUBLIC Catch2::Catch2WithMain)

set_property(TARGET TestNUClearRoles PROPERTY FOLDER "nuclear/tests")

# Add the test
add_test(
  NAME TestNUClearRoles
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/TestNUClearRoles
)
