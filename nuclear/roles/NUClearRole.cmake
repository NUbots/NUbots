# We need python!
find_package(PythonInterp 3 REQUIRED)
find_package(NUClear REQUIRED)

# We use threading
find_package(Threads REQUIRED)

function(NUCLEAR_ROLE)

  # Store our role_modules in a sane variable
  set(role_modules ${ARGN})

  # Custom command that specifies how to generate ${role}.cpp
  add_custom_command(
    OUTPUT "${role}.cpp"
    COMMAND ${PYTHON_EXECUTABLE} ARGS "${CMAKE_CURRENT_SOURCE_DIR}/generate_role.py" "${role}.cpp"
            "${NUCLEAR_ROLE_BANNER_FILE}" "${PROJECT_SOURCE_DIR}/${NUCLEAR_MODULE_DIR}" ${role_modules}
    COMMENT "Generating the ${role} role"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/generate_role.py" ${NUCLEAR_ROLE_BANNER_FILE}
  )

  # Remove the :: from each module to make a valid target name for the module
  string(REPLACE "::" "" role_module_targets "${role_modules}")

  # Build our executable from the generated role
  add_executable(${role} "${role}.cpp")

  target_include_directories(${role} PRIVATE "${PROJECT_SOURCE_DIR}/${NUCLEAR_MODULE_DIR}")

  # Link to the roles module libraries and the shared utility and extension libraries
  target_link_libraries(${role} Threads::Threads)
  target_link_libraries(${role} ${role_module_targets})
  target_link_libraries(${role} NUClear::nuclear)

  # Set our output directory to be bin
  set_target_properties(${role} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin/")

  # IDE folder
  set_target_properties(${role} PROPERTIES FOLDER "roles/")

  # Store the used NUClear modules on the target as a property This can be used later in scripts to work out what
  # modules are used in the role
  set_target_properties(${role} PROPERTIES NUCLEAR_MODULES "${role_modules}")

  # * We add to the global cache variable here that contains all of the module we are using
  # * Elsewhere, this is used to include the directories for these in order to build them
  set(NUCLEAR_MODULES
      ${NUCLEAR_MODULES} ${role_modules}
      CACHE INTERNAL "A list of the modules in use by the system" FORCE
  )
endfunction(NUCLEAR_ROLE)
