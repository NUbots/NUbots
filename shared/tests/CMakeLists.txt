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

  add_executable(testshared "${CMAKE_SOURCE_DIR}/${NUCLEAR_SHARED_DIR}/tests/test.cpp" ${test_src} ${test_data})
  target_link_libraries(testshared PRIVATE nuclear::message nuclear::utility)

  set_target_properties(testshared PROPERTIES FOLDER "shared/tests")

  # Add the test
  add_test(
    NAME testshared
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND ${CMAKE_CURRENT_BINARY_DIR}/testshared
  )
endif()
