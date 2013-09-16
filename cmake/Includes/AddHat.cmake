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

    INCLUDE_DIRECTORIES("${CMAKE_BINARY_DIR}/modules/")

    STRING(REPLACE "::" "" HAT_MODULES_TARGETS "${HAT_MODULES}")

    ADD_EXECUTABLE("${HAT_NAME}" "${CMAKE_BINARY_DIR}/hats/${HAT_NAME}.cpp")
    TARGET_LINK_LIBRARIES(${HAT_NAME} ${HAT_MODULES_TARGETS} ${NUBOTS_SHARED_LIBRARIES})

    SET(hats ${hats} ${HAT_NAME} PARENT_SCOPE)
    SET(modules ${modules} ${HAT_MODULES} PARENT_SCOPE)
ENDFUNCTION(ADD_HAT)
