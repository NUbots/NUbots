INCLUDE(CMakeParseArguments)
FUNCTION(NUCLEAR_MODULE)

    # Get our relative module path
    FILE(RELATIVE_PATH module_path ${NUCLEAR_MODULE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

    # Fix windows paths
    STRING(REPLACE "\\" "/" module_name "${module_path}")

    # Keep our modules path for grouping later
    SET(module_path "module/${module_name}")

    STRING(REPLACE "/" "" module_name "${module_name}")

    SET(options, "")
    SET(oneValueArgs "NAME")
    SET(multiValueArgs "INCLUDES" "LIBRARIES" "SOURCES")
    CMAKE_PARSE_ARGUMENTS(MODULE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    # Find all our source files
    FILE(GLOB_RECURSE src "${CMAKE_CURRENT_SOURCE_DIR}/src/**.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/**.h")

    # Get our data files
    FILE(GLOB_RECURSE data_files "${CMAKE_CURRENT_SOURCE_DIR}/data/**")

    # Process the data files
    FOREACH(data_file ${data_files})

        # Calculate the Output Directory
        FILE(RELATIVE_PATH output_file "${CMAKE_CURRENT_SOURCE_DIR}/data" ${data_file})
        SET(output_file "${CMAKE_BINARY_DIR}/${output_file}")

        # Add the file we will generate to our output
        LIST(APPEND data "${output_file}")

        # Create the required folder
        GET_FILENAME_COMPONENT(output_folder ${output_file} DIRECTORY)
        FILE(MAKE_DIRECTORY ${output_folder})

        # Create symlinks to the files
        ADD_CUSTOM_COMMAND(
            OUTPUT ${output_file}
            COMMAND ${CMAKE_COMMAND} -E create_symlink ${data_file} ${output_file}
            DEPENDS ${data_file}
            COMMENT "Creating symbolic link for file ${data_file}"
        )

    ENDFOREACH(data_file)

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

    # Add all our code to a library and if we are doing a shared build make it a shared library
    IF(NUCLEAR_SHARED_BUILD)
        ADD_LIBRARY(${module_name} SHARED ${src} ${MODULE_SOURCES} ${data} ${data_files})
        SET_PROPERTY(TARGET ${module_name} PROPERTY LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/lib")
    ELSE()
        ADD_LIBRARY(${module_name} STATIC ${src} ${MODULE_SOURCES} ${data} ${data_files})
    ENDIF()

    TARGET_LINK_LIBRARIES(${module_name} ${NUCLEAR_UTILITY_LIBRARIES} ${NUCLEAR_MESSAGE_LIBRARIES} ${NUCLEAR_EXTENSION_LIBRARIES} ${MODULE_LIBRARIES} ${NUCLEAR_LIBRARY})

    # Put it in an IDE group for shared
    SET_PROPERTY(TARGET ${module_name} PROPERTY FOLDER ${module_path})

    # If we are doing tests then build the tests for this
    IF(BUILD_TESTS)
        # Set a different name for our test module
        SET(test_module_name "Test${module_name}")

        # Rebuild our sources using the test module
        FILE(GLOB_RECURSE test_src "tests/**.cpp" "tests/**.h")
        ADD_EXECUTABLE(${test_module_name} ${test_src})
        TARGET_LINK_LIBRARIES(${test_module_name} ${module_name} ${LIBRARIES})

        SET_PROPERTY(TARGET ${test_module_name} PROPERTY FOLDER "modules/tests")

        # Add the test
        ADD_TEST(${test_module_name} ${test_module_name})

    ENDIF()

ENDFUNCTION(NUCLEAR_MODULE)
