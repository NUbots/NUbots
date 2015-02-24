# We need python!
FIND_PACKAGE(PythonInterp REQUIRED)

FUNCTION(ADD_ROLE)

    # Store our role_modules in a sane variable
    SET(role_modules ${ARGN})

    # Custom command that specifies how to generate ${role}.cpp
    ADD_CUSTOM_COMMAND(
        OUTPUT "${CMAKE_BINARY_DIR}/roles/${role}.cpp"
        COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_SOURCE_DIR}/cmake/Scripts/generate_role.py" "${CMAKE_BINARY_DIR}/roles/${role}.cpp" ${role_modules}
        COMMENT "Generating the ${role} role"
        DEPENDS "${CMAKE_SOURCE_DIR}/cmake/Scripts/generate_role.py")

    INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}")
    INCLUDE_DIRECTORIES("${CMAKE_BINARY_DIR}")

    STRING(REPLACE "::" "" role_module_targets "${role_modules}")

    ADD_EXECUTABLE("${role}" "${CMAKE_BINARY_DIR}/roles/${role}.cpp")
    TARGET_LINK_LIBRARIES(${role} ${role_module_targets} ${NUBOTS_SHARED_LIBRARIES})

    # Set our output directory to be bin
    SET_PROPERTY(TARGET ${role} PROPERTY RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/")

    # IDE folder
    SET_PROPERTY(TARGET ${role} PROPERTY FOLDER "roles/")

    #  Store the used NUClear modules on the target
    SET_PROPERTY(TARGET ${role} PROPERTY NUCLEAR_MODULES ${role_modules})

    SET(NUCLEAR_MODULES ${NUCLEAR_MODULES} ${role_modules} CACHE INTERNAL "A list of the modules in use by the system" FORCE)
ENDFUNCTION(ADD_ROLE)
