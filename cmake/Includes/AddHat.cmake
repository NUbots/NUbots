FUNCTION(ADD_HAT)
    SET(options, "")
    SET(oneValueArgs "NAME")
    SET(multiValueArgs "MODULES")
    CMAKE_PARSE_ARGUMENTS(HAT "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    # Custom command that specifies how to generate ${HAT_NAME}.cpp
    ADD_CUSTOM_COMMAND(
        OUTPUT "${CMAKE_BINARY_DIR}/hats/${HAT_NAME}.cpp"
        COMMAND "${NUBOTS_SCRIPTS_DIR}/generate.sh" "${CMAKE_BINARY_DIR}/hats/${HAT_NAME}.cpp" ${HAT_MODULES}
        COMMENT "Generating the ${HAT_NAME} hat"
        DEPENDS "${NUBOTS_SCRIPTS_DIR}/generate.sh")

    # Each hat wants to access a number of modules. We're going to set our
    # include directories to include the source directory of each module
    # avoid relative paths in our modules which makes them easier
    # to move around.
    #FOREACH(module ${HAT_MODULES})
    #    STRING(REPLACE "::" "/" module "${module}")
    #    INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/modules/${module}/src")
    #ENDFOREACH()

    INCLUDE_DIRECTORIES("${CMAKE_BINARY_DIR}/modules/")

    STRING(REPLACE "::" "" HAT_MODULES_TARGETS "${HAT_MODULES}")

    ADD_EXECUTABLE("${HAT_NAME}" "${CMAKE_BINARY_DIR}/hats/${HAT_NAME}.cpp")
    TARGET_LINK_LIBRARIES(${HAT_NAME} ${HAT_MODULES_TARGETS} ${NUBOTS_SHARED_LIBRARIES})

    SET(hats ${hats} ${HAT_NAME} PARENT_SCOPE)
    SET(modules ${modules} ${HAT_MODULES} PARENT_SCOPE)
ENDFUNCTION(ADD_HAT)
