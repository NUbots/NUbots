# We need python!
FIND_PACKAGE(PythonInterp REQUIRED)

FUNCTION(NUCLEAR_ROLE)

    # Store our role_modules in a sane variable
    SET(role_modules ${ARGN})

    # Include our messages extensions and utilty folders
    INCLUDE_DIRECTORIES(${NUCLEAR_MESSAGE_INCLUDE_DIRS})
    INCLUDE_DIRECTORIES(${NUCLEAR_UTILITY_INCLUDE_DIRS})
    INCLUDE_DIRECTORIES(${NUCLEAR_EXTENSION_INCLUDE_DIRS})

    # Custom command that specifies how to generate ${role}.cpp
    ADD_CUSTOM_COMMAND(
        OUTPUT "${role}.cpp"
        COMMAND ${PYTHON_EXECUTABLE}
        ARGS    "${CMAKE_CURRENT_SOURCE_DIR}/generate_role.py"
                "${role}.cpp"
                ${NUCLEAR_ROLE_BANNER_FILE}
                ${NUCLEAR_MODULE_DIR}
                ${role_modules}
        COMMENT "Generating the ${role} role"
        DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/generate_role.py" ${NUCLEAR_ROLE_BANNER_FILE})

    # The role cpp files are generated
    SET_SOURCE_FILES_PROPERTIES(${role}.cpp PROPERTIES GENERATED TRUE)

    # Remove the :: from each module to make a valid target name for the module
    STRING(REPLACE "::" "" role_module_targets "${role_modules}")

    # Build our executable from the generated role
    ADD_EXECUTABLE(${role} "${role}.cpp")

    # Link to the roles module libraries and the shared utility and extension libraries
    TARGET_LINK_LIBRARIES(${role} ${role_module_targets} ${NUClear_LIBRARIES} ${NUCLEAR_ADDITIONAL_SHARED_LIBRARIES})

    FOREACH(module_target ${role_module_targets})
        INCLUDE_DIRECTORIES($<TARGET_PROPERTY:${module_target},INCLUDE_DIRECTORIES>)
    ENDFOREACH(module_target)

    # Set our output directory to be bin
    SET_PROPERTY(TARGET ${role} PROPERTY RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/")

    # IDE folder
    SET_PROPERTY(TARGET ${role} PROPERTY FOLDER "roles/")

    # Store the used NUClear modules on the target as a property
    # This can be used later in scripts to work out what modules are used in the role
    SET_PROPERTY(TARGET ${role} PROPERTY NUCLEAR_MODULES ${role_modules})

    # We add to the global cache variable here that contains all of the module we are using
    # Elsewhere, this is used to include the directories for these in order to build them
    SET(NUCLEAR_MODULES ${NUCLEAR_MODULES} ${role_modules} CACHE INTERNAL "A list of the modules in use by the system" FORCE)
ENDFUNCTION(NUCLEAR_ROLE)
