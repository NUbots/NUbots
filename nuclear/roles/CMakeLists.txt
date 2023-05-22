# Get our role directories
set(role_source_dir "${PROJECT_SOURCE_DIR}/${NUCLEAR_ROLES_DIR}")
set(role_binary_dir "${PROJECT_BINARY_DIR}/${NUCLEAR_ROLES_DIR}")

# This gets all of our role files
file(GLOB_RECURSE role_files "${role_source_dir}/*.role")

# Backup the state of all the role variables before we re read them. This is needed so we can delete roles that no
# longer exist
get_cmake_property(vars CACHE_VARIABLES)
foreach(var ${vars})
  if(var MATCHES "^ROLE_[a-zA-Z0-9_-]+")
    set("BACKUP_${var}" ${${var}})
    unset(${var} CACHE)
  endif()
endforeach(var)

# We need the NUClear role function here so it can be used in the role files
include(NUClearRole.cmake)

# Clear our NUClear modules variable so we can repopulate it
unset(NUCLEAR_MODULES CACHE)

# Clear our NUClear roles variable so we can repopulate it
unset(NUCLEAR_ROLES CACHE)

# Find all of our role files and set our NUClear roles based on the ROLES cache variables that are set to true. This
# makes it so that only the roles that are enabled will be built
foreach(role_file ${role_files})
  # Get path to role and the role name and add it to our list of roles
  file(RELATIVE_PATH role_path ${role_source_dir} ${role_file})
  get_filename_component(role_name ${role_path} NAME_WE)

  # Convert path to underscore separated folders
  get_filename_component(role_path ${role_path} DIRECTORY)

  # Join the role name with the role path again
  if(role_path)
    string(CONCAT role_var "ROLE_" ${role_path} "-" ${role_name})
  else()
    string(CONCAT role_var "ROLE_" ${role_name})
  endif()

  # If we had a backup as to if this role is enabled use it. Otherwise default to true.
  if(DEFINED "BACKUP_${role_var}")
    option("${role_var}" "Build the ${role_var}" ${BACKUP_${role_var}})
  else()
    option("${role_var}" "Build the ${role_var}" ON)
  endif()

  if(${role_var})
    string(SUBSTRING ${role_var} 5 -1 role)
    set(NUCLEAR_ROLES
        ${NUCLEAR_ROLES} ${role}
        CACHE INTERNAL "A list of the roles that are enabled to be built." FORCE
    )

    # Note that the local variables in this file are available in the NUClear role function
    include("${role_source_dir}/${role_path}/${role_name}.role")
  endif()
endforeach()

# If we want to build all the modules, we will list them all in the required format then create the role manually
if(BUILD_ALL_MODULES)

  # ${role} is the name of the role created when you call nuclear_role()
  set(role "allmodules")

  # * We find all the modules by finding all CMakeLists.txt files in the module directory
  # * A directory containing a CMakeLists.txt file in the module directory is assumed to be a module
  file(
    GLOB_RECURSE module_cmakelists
    RELATIVE "${PROJECT_SOURCE_DIR}/${NUCLEAR_MODULE_DIR}"
    "${PROJECT_SOURCE_DIR}/${NUCLEAR_MODULE_DIR}/**/CMakeLists.txt"
  )

  # The paths are those of the CMakeLists.txt files, so we make them the expected format here
  foreach(module ${module_cmakelists})
    # Strip the trailing filename to get the module directory
    string(REPLACE "/CMakeLists.txt" "" module ${module})
    # Replace path separators with the namespace-style colon pairs nuclear_role() expects
    string(REPLACE "/" "::" module ${module})
    # Add the resulting string to the list of all modules
    list(APPEND ALL_MODULES ${module})
  endforeach(module ${module_cmakelists})

  # Actually generate the role with the module names
  nuclear_role(${ALL_MODULES})
endif()
