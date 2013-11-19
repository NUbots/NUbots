FUNCTION(ADD_ROLE)
    SET(options, "")
    SET(oneValueArgs "NAME")
    SET(multiValueArgs "MODULES")
    CMAKE_PARSE_ARGUMENTS(ROLE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    # Custom command that specifies how to generate ${ROLE_NAME}.cpp
    ADD_CUSTOM_COMMAND(
        OUTPUT "${CMAKE_BINARY_DIR}/roles/${ROLE_NAME}.cpp"
        COMMAND "${NUBOTS_SCRIPTS_DIR}/generate.sh" "${CMAKE_BINARY_DIR}/roles/${ROLE_NAME}.cpp" ${ROLE_MODULES}
        COMMENT "Generating the ${ROLE_NAME} role"
        DEPENDS "${NUBOTS_SCRIPTS_DIR}/generate.sh")

    INCLUDE_DIRECTORIES("${CMAKE_BINARY_DIR}/modules/")

    STRING(REPLACE "::" "" ROLE_MODULES_TARGETS "${ROLE_MODULES}")

    ADD_EXECUTABLE("${ROLE_NAME}" "${CMAKE_BINARY_DIR}/roles/${ROLE_NAME}.cpp")
    TARGET_LINK_LIBRARIES(${ROLE_NAME} ${ROLE_MODULES_TARGETS} ${NUBOTS_SHARED_LIBRARIES})

    SET(roles ${roles} ${ROLE_NAME} PARENT_SCOPE)
    SET(modules ${modules} ${ROLE_MODULES} PARENT_SCOPE)
ENDFUNCTION(ADD_ROLE)
