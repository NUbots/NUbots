# We use NUClearModule
include(NUClearModule.cmake)

# Remove any duplicate modules (used in more then one role)
set(modules ${NUCLEAR_MODULES})
list(REMOVE_DUPLICATES modules)
set(NUCLEAR_MODULES ${modules} CACHE INTERNAL "A list of the modules in use by the system" FORCE)

# Clear our list of data files
unset(NUCLEAR_MODULE_DATA_FILES CACHE)

# Build each of our modules and their corresponding tests
foreach(module ${NUCLEAR_MODULES})
  string(REPLACE "::" "/" module_path "${module}")
  string(REPLACE "::" "" module_name "${module}")

  # Add the subdirectories
  add_subdirectory("${PROJECT_SOURCE_DIR}/${NUCLEAR_MODULE_DIR}/${module_path}"
                   "${PROJECT_BINARY_DIR}/${NUCLEAR_MODULE_DIR}/${module_path}")
endforeach()
