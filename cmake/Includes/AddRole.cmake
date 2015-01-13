FUNCTION(ADD_ROLE)

    # Store our role_modules in a sane variable
    SET(role_modules ${ARGN})

    # Custom command that specifies how to generate ${role}.cpp
    ADD_CUSTOM_COMMAND(
        OUTPUT "${CMAKE_BINARY_DIR}/roles/${role}.cpp"
        COMMAND "${CMAKE_SOURCE_DIR}/cmake/Scripts/generate.py" "${CMAKE_BINARY_DIR}/roles/${role}.cpp" ${role_modules}
        COMMENT "Generating the ${role} role"
        DEPENDS "${CMAKE_SOURCE_DIR}/cmake/Scripts/generate.py")

    INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}")
    INCLUDE_DIRECTORIES("${CMAKE_BINARY_DIR}")

    STRING(REPLACE "::" "" role_module_targets "${role_modules}")

    ADD_EXECUTABLE("${role}" "${CMAKE_BINARY_DIR}/roles/${role}.cpp")
    TARGET_LINK_LIBRARIES(${role} ${role_module_targets} ${NUBOTS_SHARED_LIBRARIES})
    SET_PROPERTY(TARGET ${role} PROPERTY RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/")

    SET_PROPERTY(TARGET ${role} PROPERTY FOLDER "roles/")

    SET(roles ${roles} ${role} PARENT_SCOPE)
    SET(modules ${modules} ${role_modules} PARENT_SCOPE)
ENDFUNCTION(ADD_ROLE)
