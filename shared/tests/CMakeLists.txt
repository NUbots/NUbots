find_package(Catch2 REQUIRED)

# This gets all of our source files
file(GLOB_RECURSE test_src "*/**.cpp" "*/**.hpp")

if(test_src)
  # Check for module test data
  file(GLOB_RECURSE test_data_files "${CMAKE_CURRENT_SOURCE_DIR}/data/**")
  foreach(test_data_file ${test_data_files})
    # Calculate the Output Directory
    file(RELATIVE_PATH output_file "${CMAKE_CURRENT_SOURCE_DIR}/data" ${test_data_file})
    set(output_file "${PROJECT_BINARY_DIR}/tests/${output_file}")

    # Add the file we will generate to our output
    list(APPEND test_data "${output_file}")

    # Create the required folder
    get_filename_component(output_folder ${output_file} DIRECTORY)
    file(MAKE_DIRECTORY ${output_folder})

    # Copy across the files
    add_custom_command(
      OUTPUT ${output_file}
      COMMAND ${CMAKE_COMMAND} -E copy ${test_data_file} ${output_file}
      DEPENDS ${test_data_file}
      COMMENT "Copying updated test data file ${test_data_file}"
    )
  endforeach(test_data_file)

  add_executable(TestShared ${test_src} ${test_data})
  target_link_libraries(TestShared nuclear::message)
  target_link_libraries(TestShared nuclear::utility)
  target_link_libraries(TestShared Catch2::Catch2WithMain)

  set_target_properties(TestShared PROPERTIES FOLDER "shared/tests")

  # Add the test
  add_test(
    NAME TestShared
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND ${CMAKE_CURRENT_BINARY_DIR}/TestShared
  )
endif()
