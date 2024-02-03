#[[
MIT License

Copyright (c) 2019 NUbots

This file is part of the NUbots codebase.
See https://github.com/NUbots/NUbots for further info.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
]]
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
