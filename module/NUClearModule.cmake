include(CMakeParseArguments)

function(NUCLEAR_MODULE)

  get_filename_component(module_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)

  # Get our relative module path
  file(RELATIVE_PATH module_target_name "${PROJECT_SOURCE_DIR}/${NUCLEAR_MODULE_DIR}" ${CMAKE_CURRENT_SOURCE_DIR})

  # Fix windows paths
  string(REPLACE "\\" "/" module_target_name "${module_path}")

  # Keep our modules path for grouping later
  set(module_path "module/${module_target_name}")

  # Stip out slashes to make it a valid target name
  string(REPLACE "/" "" module_target_name "${module_target_name}")

  # Parse our input arguments
  set(options, "")
  set(oneValueArgs "LANGUAGE")
  set(multiValueArgs "INCLUDES" "LIBRARIES" "SOURCES" "DATA_FILES")
  cmake_parse_arguments(MODULE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  # Include our own source and binary directories
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
  include_directories(${CMAKE_CURRENT_BINARY_DIR}/src)

  # Include our messages extensions and utility folders
  include_directories(${NUCLEAR_MESSAGE_INCLUDE_DIRS})
  include_directories(${NUCLEAR_UTILITY_INCLUDE_DIRS})
  include_directories(${NUCLEAR_EXTENSION_INCLUDE_DIRS})

  # Include any directories passed into the function
  include_directories(SYSTEM ${MODULE_INCLUDES})

  # Include any directories used in messages utilities and extensions
  foreach(lib ${NUCLEAR_MESSAGE_LIBRARIES} ${NUCLEAR_UTILITY_LIBRARIES} ${NUCLEAR_EXTENSION_LIBRARIES})
    include_directories($<TARGET_PROPERTY:${lib},INCLUDE_DIRECTORIES>)
  endforeach(lib)

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
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/src/${module_name}.h" "${CMAKE_CURRENT_BINARY_DIR}/src/${module_name}.cpp"
      COMMAND
        ${CMAKE_COMMAND}
        ARGS
        -E
        env
        PYTHONPATH="${PROJECT_BINARY_DIR}/python/nuclear/"
        NUCLEAR_MODULE_DIR="${PROJECT_SOURCE_DIR}/${NUCLEAR_MODULE_DIR}"
        ${PYTHON_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/src/${module_name}.py"
      WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/src"
      DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/${module_name}.py" ${NUCLEAR_MESSAGE_LIBRARIES}
      COMMENT "Generating bindings for python module ${module_name}"
    )

    set(src "${CMAKE_CURRENT_BINARY_DIR}/src/${module_name}.h" "${CMAKE_CURRENT_BINARY_DIR}/src/${module_name}.cpp"
            ${python_code})
  endif()

  # ####################################################################################################################
  # Data files #
  # ####################################################################################################################

  # Get our data files
  file(GLOB_RECURSE data_files "${CMAKE_CURRENT_SOURCE_DIR}/data/**")

  # Process the data files
  foreach(data_file ${data_files})

    # Calculate the Output Directory
    file(RELATIVE_PATH output_file "${CMAKE_CURRENT_SOURCE_DIR}/data" ${data_file})
    set(output_file "${PROJECT_BINARY_DIR}/${output_file}")

    # Add the file we will generate to our output
    list(APPEND data "${output_file}")

    # Create the required folder
    get_filename_component(output_folder ${output_file} DIRECTORY)
    file(MAKE_DIRECTORY ${output_folder})

    # Copy across the files
    add_custom_command(
      OUTPUT ${output_file}
      COMMAND ${CMAKE_COMMAND} -E copy ${data_file} ${output_file}
      DEPENDS ${data_file}
      COMMENT "Copying updated data file ${data_file}"
    )

    set(
      NUCLEAR_MODULE_DATA_FILES
      ${NUCLEAR_MODULE_DATA_FILES} ${output_file}
      CACHE INTERNAL "A list of all the data files that were generated by modules" FORCE
    )
  endforeach(data_file)

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

    set(
      NUCLEAR_MODULE_DATA_FILES
      ${NUCLEAR_MODULE_DATA_FILES} ${output_file}
      CACHE INTERNAL "A list of all the data files that were generated by modules" FORCE
    )
  endforeach(data_file)

  # ####################################################################################################################
  # Build into library #
  # ####################################################################################################################

  # Add all our code to a library and if we are doing a shared build make it a shared library
  set(sources ${src} ${MODULE_SOURCES} ${data})

  if(NUCLEAR_SHARED_BUILD)
    add_library(${module_target_name} SHARED ${sources})
    set_property(TARGET ${module_target_name} PROPERTY LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin/lib")
  else()
    add_library(${module_target_name} STATIC ${sources})
  endif()

  target_link_libraries(
    ${module_target_name}
    ${NUCLEAR_UTILITY_LIBRARIES}
    ${NUCLEAR_MESSAGE_LIBRARIES}
    ${NUCLEAR_EXTENSION_LIBRARIES}
    ${MODULE_LIBRARIES}
    ${NUClear_LIBRARIES}
  )

  # Put it in an IDE group for shared
  set_property(TARGET ${module_target_name} PROPERTY FOLDER ${module_path})

  # ####################################################################################################################
  # Testing #
  # ####################################################################################################################

  # If we are doing tests then build the tests for this
  if(BUILD_TESTS)
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
      add_executable(${test_module_target_name} ${test_src})
      target_link_libraries(${test_module_target_name} ${module_target_name} ${MODULE_TEST_LIBRARIES}
                            ${NUCLEAR_TEST_LIBRARIES})

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
