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

  # Get separate components of role path to properly set the output folder, target name and executable
  get_filename_component(role_name ${role} NAME)
  cmake_path(GET role PARENT_PATH role_folder)

  # Target name must be unique and can't be a path, so if this is in a folder then change the slashes to underscores
  string(REPLACE "/" "_" target_name "${role}")

  # Build our executable from the generated role
  add_executable(${target_name} "${role_name}.cpp")
  SET_TARGET_PROPERTIES(${target_name} PROPERTIES OUTPUT_NAME ${role_name})

  target_include_directories(${target_name} PRIVATE "${PROJECT_SOURCE_DIR}/${NUCLEAR_MODULE_DIR}")

  # Link to the roles module libraries and the shared utility and extension libraries
  target_link_libraries(${target_name} Threads::Threads)
  target_link_libraries(${target_name} ${role_module_targets})
  target_link_libraries(${target_name} NUClear::nuclear)

  # Set our output directory to be bin
  set_target_properties(${target_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin/${role_folder}")

  # IDE folder
  set_target_properties(${target_name} PROPERTIES FOLDER "roles/${role_folder}")

  # Store the used NUClear modules on the target as a property This can be used later in scripts to work out what
  # modules are used in the role
  set_target_properties(${target_name} PROPERTIES NUCLEAR_MODULES "${role_modules}")

  # * We add to the global cache variable here that contains all of the module we are using
  # * Elsewhere, this is used to include the directories for these in order to build them
  set(NUCLEAR_MODULES
      ${NUCLEAR_MODULES} ${role_modules}
      CACHE INTERNAL "A list of the modules in use by the system" FORCE
  )
endfunction(NUCLEAR_ROLE)
