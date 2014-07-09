
IF(SHARED_BUILD)
    # use, i.e. don't skip the full RPATH for the build tree
    SET(CMAKE_SKIP_BUILD_RPATH FALSE)

    # Build the RPATH into the binary before install
    SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

    # the RPATH to be used
    SET(CMAKE_INSTALL_RPATH "lib/" "../lib/" "bin/lib")
ENDIF()

FUNCTION(NUCLEAR_MODULE)

    STRING(REGEX REPLACE "^.*modules/(.+)$" "\\1;" module_name "${CMAKE_CURRENT_SOURCE_DIR}")
    STRING(REPLACE "\\" "/" module_name "${module_name}")

    # Keep our modules path for grouping later
    SET(module_path modules/${module_name})

    STRING(REPLACE "/" "" module_name "${module_name}")

    # Set our variables initial values
    SET(INCLUDES "")
    SET(LIBRARIES "")
    SET(ISLIBS "FALSE")
    SET(ISINCS "FALSE")
    SET(ISSRCS "FALSE")

    # Loop through all our args
    FOREACH(arg ${ARGV})
        # Work out if we are in an INCLUDES or LIBRARIES section
        IF(${arg} STREQUAL "INCLUDES")
            SET(ISLIBS "FALSE")
            SET(ISINCS "TRUE")
            SET(ISSRCS "FALSE")
        ELSEIF(${arg} STREQUAL "LIBRARIES")
            SET(ISLIBS "TRUE")
            SET(ISINCS "FALSE")
            SET(ISSRCS "FALSE")

        ELSEIF(${arg} STREQUAL "SOURCES")
            SET(ISLIBS "FALSE")
            SET(ISINCS "FALSE")
            SET(ISSRCS "TRUE")

        # Store this argument in the correct list
        ELSE()
            IF(ISLIBS)
                SET(LIBRARIES ${LIBRARIES} ${arg})
            ELSEIF(ISINCS)
                SET(INCLUDES ${INCLUDES} ${arg})
            ELSEIF(ISSRCS)
                SET(SOURCES ${SOURCES} ${arg})
            ELSE()
                MESSAGE(FATAL_ERROR "Modules take LIBRARIES, INCLUDES and SOURCES only")
            ENDIF()
        ENDIF()
    ENDFOREACH()

    # Find all our files
    FILE(GLOB_RECURSE src "${CMAKE_CURRENT_SOURCE_DIR}/src/**.cpp" , "${CMAKE_CURRENT_SOURCE_DIR}/src/**.h")

    # Get our configuration files
    FILE(GLOB_RECURSE config_files "config/**")

    FOREACH(config_file ${config_files})

        # Calculate the Output Directory
        FILE(RELATIVE_PATH output_file "${CMAKE_CURRENT_SOURCE_DIR}/config" ${config_file})
        SET(output_file "${CMAKE_BINARY_DIR}/config/${output_file}")

        # Add the two files we will generate to our output
        LIST(APPEND configuration "${output_file}")

        # Copy configuration files over as needed
        ADD_CUSTOM_COMMAND(
            OUTPUT ${output_file}
            COMMAND ${CMAKE_COMMAND} -E copy ${config_file} ${output_file}
            DEPENDS ${config_file}
            COMMENT "Copying updated configuration file ${config_file}"
        )

    ENDFOREACH()

    # Include our own src dir and the shared dirs
    INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/src)
    INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}/src)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/shared)
    INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/shared)

    # Include any directories passed into the function
    INCLUDE_DIRECTORIES(${INCLUDES})

    # Add all our code to a library
    IF(SHARED_BUILD)
        ADD_LIBRARY(${module_name} SHARED ${src} ${SOURCES} ${configuration})
        SET_PROPERTY(TARGET ${module_name} PROPERTY LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/lib")
    ELSE()
        ADD_LIBRARY(${module_name} STATIC ${src} ${SOURCES} ${configuration})
    ENDIF()

    TARGET_LINK_LIBRARIES(${module_name} ${NUBOTS_SHARED_LIBRARIES} ${LIBRARIES})

    SET_PROPERTY(TARGET ${module_name} PROPERTY LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/lib")

    # Put it in an IDE group for shared
    SET_PROPERTY(TARGET ${module_name} PROPERTY FOLDER ${module_path})

    # If we are doing tests then build the tests for this
    IF(BUILD_TESTS)
        # Set a different name for our test module
        SET(test_module_name "Test${module_name}")

        # Rebuild our sources using the test module
        FILE(GLOB_RECURSE test_src "tests/**.cpp" , "tests/**.h")
        ADD_EXECUTABLE(${test_module_name} ${test_src})
        TARGET_LINK_LIBRARIES(${test_module_name} ${module_name} ${NUBOTS_SHARED_LIBRARIES} ${LIBRARIES})


        SET_PROPERTY(TARGET ${test_module_name} PROPERTY FOLDER "modules/tests")

    ENDIF()

ENDFUNCTION(NUCLEAR_MODULE)
