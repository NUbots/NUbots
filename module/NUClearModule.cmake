INCLUDE(CMakeParseArguments)

FUNCTION(NUCLEAR_MODULE)

    GET_FILENAME_COMPONENT(module_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)

    # Get our relative module path
    FILE(RELATIVE_PATH module_target_name "${PROJECT_SOURCE_DIR}/${NUCLEAR_MODULE_DIR}" ${CMAKE_CURRENT_SOURCE_DIR})

    # Fix windows paths
    STRING(REPLACE "\\" "/" module_target_name "${module_path}")

    # Keep our modules path for grouping later
    SET(module_path "module/${module_target_name}")

    # Stip out slashes to make it a valid target name
    STRING(REPLACE "/" "" module_target_name "${module_target_name}")

    # Parse our input arguments
    SET(options, "")
    SET(oneValueArgs "LANGUAGE")
    SET(multiValueArgs "INCLUDES" "LIBRARIES" "SOURCES")
    CMAKE_PARSE_ARGUMENTS(MODULE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    # Include our own source and binary directories
    INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/src)
    INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}/src)

    # Include our messages extensions and utilty folders
    INCLUDE_DIRECTORIES(${NUCLEAR_MESSAGE_INCLUDE_DIRS})
    INCLUDE_DIRECTORIES(${NUCLEAR_UTILITY_INCLUDE_DIRS})
    INCLUDE_DIRECTORIES(${NUCLEAR_EXTENSION_INCLUDE_DIRS})

    # Include any directories passed into the function
    INCLUDE_DIRECTORIES(SYSTEM ${MODULE_INCLUDES})

    # Include any directories used in messages utilities and extensions
    FOREACH(lib ${NUCLEAR_MESSAGE_LIBRARIES} ${NUCLEAR_UTILITY_LIBRARIES} ${NUCLEAR_EXTENSION_LIBRARIES})
        INCLUDE_DIRECTORIES($<TARGET_PROPERTY:${lib},INCLUDE_DIRECTORIES>)
    ENDFOREACH(lib)

    #########################
    # Find or generate code #
    #########################

    # CPP Code
    IF(NOT MODULE_LANGUAGE OR MODULE_LANGUAGE STREQUAL "CPP")

        # A CPP module just use sources in this directory
        FILE(GLOB_RECURSE src
            "${CMAKE_CURRENT_SOURCE_DIR}/src/**.cpp"
            "${CMAKE_CURRENT_SOURCE_DIR}/src/**.cc"
            "${CMAKE_CURRENT_SOURCE_DIR}/src/**.c"
            "${CMAKE_CURRENT_SOURCE_DIR}/src/**.hpp"
            "${CMAKE_CURRENT_SOURCE_DIR}/src/**.ipp"
            "${CMAKE_CURRENT_SOURCE_DIR}/src/**.h")

    # Python Code
    ELSEIF(MODULE_LANGUAGE STREQUAL "PYTHON")

        FIND_PACKAGE(PythonInterp 3 REQUIRED)
        FIND_PACKAGE(pybind11 REQUIRED)
        FIND_PACKAGE(PythonLibsNew 3 REQUIRED)

        # Now copy all our python files across to the python directory of output
        FILE(GLOB_RECURSE python_files "${CMAKE_CURRENT_SOURCE_DIR}/src/**.py")

        # Copy the python files into the build directory
        FOREACH(python_file ${python_files})

            # Calculate the output Directory
            FILE(RELATIVE_PATH output_file "${PROJECT_SOURCE_DIR}/${NUCLEAR_MODULE_DIR}" ${python_file})
            SET(output_file "${PROJECT_BINARY_DIR}/python/${output_file}")

            # Add the file we will generate to our output
            LIST(APPEND python_code ${output_file})

            # Create the required folder
            GET_FILENAME_COMPONENT(output_folder ${output_file} DIRECTORY)
            FILE(MAKE_DIRECTORY ${output_folder})

            # Copy across our file
            ADD_CUSTOM_COMMAND(
                OUTPUT ${output_file}
                COMMAND ${CMAKE_COMMAND} -E copy ${python_file} ${output_file}
                DEPENDS ${python_file}
                COMMENT "Copying updated python file ${python_file}"
            )

        ENDFOREACH(python_file)

        FILE(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/src")

        ADD_CUSTOM_COMMAND(
            OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/src/${module_name}.h"
                   "${CMAKE_CURRENT_BINARY_DIR}/src/${module_name}.cpp"
            COMMAND ${CMAKE_COMMAND}
            ARGS -E env
                PYTHONPATH="${PROJECT_BINARY_DIR}/python/nuclear/"
                NUCLEAR_MODULE_DIR="${PROJECT_SOURCE_DIR}/${NUCLEAR_MODULE_DIR}"
                LD_LIBRARY_PATH="/nubots/toolchain/lib"
                ${PYTHON_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/src/${module_name}.py"
            WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/src"
            DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/${module_name}.py"
                    ${NUCLEAR_MESSAGE_LIBRARIES}
            COMMENT "Generating bindings for python module ${module_name}")

        SET(src "${CMAKE_CURRENT_BINARY_DIR}/src/${module_name}.h"
                "${CMAKE_CURRENT_BINARY_DIR}/src/${module_name}.cpp"
                ${python_code})
    ENDIF()

    ##############
    # Data files #
    ##############

    # Get our data files
    FILE(GLOB_RECURSE data_files "${CMAKE_CURRENT_SOURCE_DIR}/data/**")

    # Process the data files
    FOREACH(data_file ${data_files})

        # Calculate the Output Directory
        FILE(RELATIVE_PATH output_file "${CMAKE_CURRENT_SOURCE_DIR}/data" ${data_file})
        SET(output_file "${PROJECT_BINARY_DIR}/${output_file}")

        # Add the file we will generate to our output
        LIST(APPEND data "${output_file}")

        # Create the required folder
        GET_FILENAME_COMPONENT(output_folder ${output_file} DIRECTORY)
        FILE(MAKE_DIRECTORY ${output_folder})

        # Copy across the files
        ADD_CUSTOM_COMMAND(
            OUTPUT ${output_file}
            COMMAND ${CMAKE_COMMAND} -E copy ${data_file} ${output_file}
            DEPENDS ${data_file}
            COMMENT "Copying updated data file ${data_file}"
        )

    ENDFOREACH(data_file)

    ######################
    # Build into library #
    ######################

    # Add all our code to a library and if we are doing a shared build make it a shared library
    IF(NUCLEAR_SHARED_BUILD)
        ADD_LIBRARY(${module_target_name} SHARED ${src} ${MODULE_SOURCES} ${data} ${data_files})
        SET_PROPERTY(TARGET ${module_target_name} PROPERTY LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin/lib")
    ELSE()
        ADD_LIBRARY(${module_target_name} STATIC ${src} ${MODULE_SOURCES} ${data} ${data_files})
    ENDIF()

    TARGET_LINK_LIBRARIES(${module_target_name} ${NUCLEAR_UTILITY_LIBRARIES} ${NUCLEAR_MESSAGE_LIBRARIES} ${NUCLEAR_EXTENSION_LIBRARIES} ${MODULE_LIBRARIES} ${NUClear_LIBRARIES})

    # Put it in an IDE group for shared
    SET_PROPERTY(TARGET ${module_target_name} PROPERTY FOLDER ${module_path})

    ###########
    # Testing #
    ###########

    # If we are doing tests then build the tests for this
    IF(BUILD_TESTS)
        # Set a different name for our test module
        SET(test_module_target_name "Test${module_target_name}")

        # Rebuild our sources using the test module
        FILE(GLOB_RECURSE test_src "tests/**.cpp" "tests/**.h")
        ADD_EXECUTABLE(${test_module_target_name} ${test_src})
        TARGET_LINK_LIBRARIES(${test_module_target_name} ${module_target_name} ${LIBRARIES})

        SET_PROPERTY(TARGET ${test_module_target_name} PROPERTY FOLDER "modules/tests")

        # Add the test
        ADD_TEST(${test_module_target_name} ${test_module_target_name})

    ENDIF()

ENDFUNCTION(NUCLEAR_MODULE)
