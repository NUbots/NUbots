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

    # Loop through all our args
    FOREACH(arg ${ARGV})
        # Work out if we are in an INCLUDES or LIBRARIES section
        IF(${arg} STREQUAL "INCLUDES")
            SET(ISLIBS "FALSE")
            SET(ISINCS "TRUE")
        ELSEIF(${arg} STREQUAL "LIBRARIES")
            SET(ISLIBS "TRUE")
            SET(ISINCS "FALSE")

        # Store this argument in the correct list
        ELSE()
            IF(ISLIBS)
                SET(LIBRARIES ${LIBRARIES} ${arg})
            ELSEIF(ISINCS)
                SET(INCLUDES ${INCLUDES} ${arg})
            ELSE()
                MESSAGE(FATAL_ERROR "Modules take LIBRARIES or INCLUDES only")
            ENDIF()
        ENDIF()
    ENDFOREACH()

    # Find all our files
    FILE(GLOB_RECURSE src "src/**.cpp" , "src/**.h")

    # Include our own src dir and the shared dirs
    INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/src)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/shared)
    INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/shared)

    # Include any directories passed into the function
    INCLUDE_DIRECTORIES(${INCLUDES})

    # Add all our code to a library
    ADD_LIBRARY(${module_name} ${src})
    TARGET_LINK_LIBRARIES(${module_name} ${NUBOTS_SHARED_LIBRARIES} ${LIBRARIES})

    # Put it in an IDE group for shared
    SET_PROPERTY(TARGET ${module_name} PROPERTY FOLDER ${module_path})

    ADD_CUSTOM_COMMAND(TARGET ${module_name} PRE_BUILD
                       COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/config ${CMAKE_BINARY_DIR}/config)

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
