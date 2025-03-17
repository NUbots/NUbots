include(CMakeParseArguments)

# We use threading and NUClear
find_package(Threads REQUIRED)
find_package(NUClear REQUIRED)

function(NUCLEAR_MODULE)

  get_filename_component(module_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)

  # Get our relative module path
  file(RELATIVE_PATH module_target_name "${PROJECT_SOURCE_DIR}/${NUCLEAR_MODULE_DIR}" ${CMAKE_CURRENT_SOURCE_DIR})

  # Fix windows paths
  string(REPLACE "\\" "/" module_target_name "${module_path}")

  # Keep our modules path for grouping later
  set(module_path "module/${module_target_name}")

  # Strip out slashes to make it a valid target name
  string(REPLACE "/" "" module_target_name "${module_target_name}")

  # Parse our input arguments
  set(options "")
  set(oneValueArgs "LANGUAGE")
  set(multiValueArgs "LIBRARIES" "SOURCES" "DATA_FILES")
  cmake_parse_arguments(MODULE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  # ####################################################################################################################
  # Find or generate code #
  # ####################################################################################################################

  # CPP Code
  if(NOT MODULE_LANGUAGE OR MODULE_LANGUAGE STREQUAL "CPP")

    # A CPP module just use sources in this directory
    file(
      GLOB_RECURSE
      src
      "${CMAKE_CURRENT_SOURCE_DIR}/src/**.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/**.cc"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/**.c"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/**.hpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/**.ipp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/**.hh"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/**.h"
    )

    # Python Code
  elseif(MODULE_LANGUAGE STREQUAL "PYTHON")

    find_package(PythonInterp 3 REQUIRED)
    find_package(pybind11 REQUIRED)
    find_package(PythonLibsNew 3 REQUIRED)

    # Now copy all our python files across to the python directory of output
    file(GLOB_RECURSE python_files "${CMAKE_CURRENT_SOURCE_DIR}/src/**.py")

    # Copy the python files into the build directory
    foreach(python_file ${python_files})

      # Calculate the output Directory
      file(RELATIVE_PATH output_file "${PROJECT_SOURCE_DIR}/${NUCLEAR_MODULE_DIR}" ${python_file})
      set(output_file "${PROJECT_BINARY_DIR}/python/${output_file}")

      # Add the file we will generate to our output
      list(APPEND python_code ${output_file})

      # Create the required folder
      get_filename_component(output_folder ${output_file} DIRECTORY)
      file(MAKE_DIRECTORY ${output_folder})

      # Copy across our file
      add_custom_command(
        OUTPUT ${output_file}
        COMMAND ${CMAKE_COMMAND} -E copy ${python_file} ${output_file}
        DEPENDS ${python_file}
        COMMENT "Copying updated python file ${python_file}"
      )

    endforeach(python_file)

    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/src")

    add_custom_command(
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/src/${module_name}.hpp" "${CMAKE_CURRENT_BINARY_DIR}/src/${module_name}.cpp"
      COMMAND
        ${CMAKE_COMMAND} ARGS -E env PYTHONPATH="${PROJECT_BINARY_DIR}/python/nuclear/"
        NUCLEAR_MODULE_DIR="${PROJECT_SOURCE_DIR}/${NUCLEAR_MODULE_DIR}" ${PYTHON_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/src/${module_name}.py"
      WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/src"
      DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/${module_name}.py" nuclear::message
      COMMENT "Generating bindings for python module ${module_name}"
    )

    set(src "${CMAKE_CURRENT_BINARY_DIR}/src/${module_name}.hpp" "${CMAKE_CURRENT_BINARY_DIR}/src/${module_name}.cpp"
            ${python_code}
    )
  endif()

  # ####################################################################################################################
  # Data files #
  # ####################################################################################################################
  # Helper function to find data files in parent directories
  function(find_parent_data_folders current_dir)
    # Check if at the module directory
    if("${current_dir}" STREQUAL "${PROJECT_SOURCE_DIR}/${NUCLEAR_MODULE_DIR}")
      return()
    endif()

    # Check for data files in this directory
    file(GLOB_RECURSE data_files "${current_dir}/data/**")
    foreach(data_file ${data_files})
      file(RELATIVE_PATH output_file "${current_dir}/data" ${data_file})
      set(output_file "${PROJECT_BINARY_DIR}/${output_file}")
      list(APPEND data "${output_file}")

      get_filename_component(output_folder ${output_file} DIRECTORY)
      file(MAKE_DIRECTORY ${output_folder})

      add_custom_command(
        OUTPUT ${output_file}
        COMMAND ${CMAKE_COMMAND} -E copy ${data_file} ${output_file}
        DEPENDS ${data_file}
        COMMENT "Copying updated data file ${data_file}"
      )

      set(NUCLEAR_MODULE_DATA_FILES
          ${NUCLEAR_MODULE_DATA_FILES} ${output_file}
          CACHE INTERNAL "A list of all the data files that were generated by modules" FORCE
      )
    endforeach(data_file)

    get_filename_component(parent_dir ${current_dir} DIRECTORY)
    find_parent_data_folders(${parent_dir})
  endfunction()

  # First get any data files in the parent directory/s
  find_parent_data_folders(${CMAKE_CURRENT_SOURCE_DIR})

  # Copy over extra data files that the module wants to install
  foreach(data_file ${MODULE_DATA_FILES})
    string(REPLACE ":" ";" data_file ${data_file})

    # Get data file name
    list(GET data_file 0 input_file)
    get_filename_component(input_file_name ${input_file} NAME)

    # Determine output file
    list(LENGTH data_file list_length)
    if(${list_length} EQUAL 1)
      set(output_file "${PROJECT_BINARY_DIR}/${input_file_name}")

    else()
      list(GET data_file 1 output_folder)
      file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/${output_folder})
      set(output_file ${PROJECT_BINARY_DIR}/${output_folder}/${input_file_name})
    endif()

    # Add the file we will generate to our output
    list(APPEND data "${output_file}")

    # Copy across the files
    add_custom_command(
      OUTPUT ${output_file}
      COMMAND ${CMAKE_COMMAND} -E copy ${input_file} ${output_file}
      DEPENDS ${input_file}
      COMMENT "Copying updated data file ${input_file}"
    )

    set(NUCLEAR_MODULE_DATA_FILES
        ${NUCLEAR_MODULE_DATA_FILES} ${output_file}
        CACHE INTERNAL "A list of all the data files that were generated by modules" FORCE
    )
  endforeach(data_file)

  # ####################################################################################################################
  # Build into library #
  # ####################################################################################################################

  # Add all our code to a library and if we are doing a shared build make it a shared library
  set(sources ${src} ${MODULE_SOURCES} ${data})

  if(NUCLEAR_LINK_TYPE STREQUAL "SHARED")
    add_library(${module_target_name} SHARED ${sources})
    set_property(TARGET ${module_target_name} PROPERTY LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin/lib")
  else()
    add_library(${module_target_name} ${NUCLEAR_LINK_TYPE} ${sources})
  endif()

  # Our source dir and binary dir are our include paths
  target_include_directories(${module_target_name} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")
  target_include_directories(${module_target_name} PUBLIC "${CMAKE_CURRENT_BINARY_DIR}/src")

  # Link to the target libraries
  target_link_libraries(${module_target_name} PUBLIC Threads::Threads)
  target_link_libraries(${module_target_name} PUBLIC NUClear::nuclear)
  target_link_libraries(${module_target_name} PUBLIC nuclear::utility nuclear::message nuclear::extension)
  target_link_libraries(${module_target_name} PUBLIC ${MODULE_LIBRARIES})

  # Put it in an IDE group for shared
  set_property(TARGET ${module_target_name} PROPERTY FOLDER ${module_path})

  # ####################################################################################################################
  # Testing #
  # ####################################################################################################################

  # If we are doing tests then build the tests for this
  if(BUILD_TESTS)
    find_package(Catch2 REQUIRED)

    # Set a different name for our test module
    set(test_module_target_name "Test${module_target_name}")

    # Rebuild our sources using the test module
    file(
      GLOB_RECURSE
      test_src
      "tests/**.cpp"
      "tests/**.cc"
      "tests/**.c"
      "tests/**.hpp"
      "tests/**.hh"
      "tests/**.h"
    )
    if(test_src)
      # Check for module test data
      file(GLOB_RECURSE test_data_files "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/**")
      foreach(test_data_file ${test_data_files})
        # Calculate the Output Directory
        file(RELATIVE_PATH output_file "${CMAKE_CURRENT_SOURCE_DIR}/tests/data" ${test_data_file})
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

      add_executable(${test_module_target_name} ${test_src} ${test_data})

      # Our source dir and binary dir are our include paths
      target_include_directories(${test_module_target_name} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")
      target_include_directories(${test_module_target_name} PUBLIC "${CMAKE_CURRENT_BINARY_DIR}/src")
      target_include_directories(${test_module_target_name} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/tests")
      target_include_directories(${test_module_target_name} PUBLIC "${CMAKE_CURRENT_BINARY_DIR}/tests")

      target_link_libraries(${test_module_target_name} ${module_target_name})
      target_link_libraries(${test_module_target_name} Catch2::Catch2WithMain)

      set_property(TARGET ${test_module_target_name} PROPERTY FOLDER "modules/tests")

      # Add the test
      add_test(
        NAME ${test_module_target_name}
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMAND ${CMAKE_CURRENT_BINARY_DIR}/${test_module_target_name}
      )

    endif()
  endif()

endfunction(NUCLEAR_MODULE)
