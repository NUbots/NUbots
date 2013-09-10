FUNCTION(ADD_HAT)
    SET(options, "")
    SET(oneValueArgs "NAME")
    SET(multiValueArgs "MODULES")
    CMAKE_PARSE_ARGUMENTS(HAT "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    # Custom command that specifies how to generate ${HAT_NAME}.cpp
    ADD_CUSTOM_COMMAND(
        OUTPUT "${CMAKE_BINARY_DIR}/hats/${HAT_NAME}.cpp"
        COMMAND "${NUBOTS_SCRIPTS_DIR}/generate.sh" "${CMAKE_BINARY_DIR}/hats/${HAT_NAME}.cpp" ${HAT_MODULES}
        COMMENT "Generating hat \"${HAT_NAME}\" with modules \"${HAT_MODULES}\""
        DEPENDS "${NUBOTS_SCRIPTS_DIR}/generate.sh")

    ADD_EXECUTABLE("${HAT_NAME}" "${CMAKE_BINARY_DIR}/hats/${HAT_NAME}.cpp")
    TARGET_LINK_LIBRARIES(${HAT_NAME} ${HAT_MODULES} ${NUBOTS_SHARED_LIBRARIES})

ENDFUNCTION(ADD_HAT)
