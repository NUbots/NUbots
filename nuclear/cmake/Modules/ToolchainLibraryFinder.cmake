include(CMakeParseArguments)
function(ToolchainLibraryFinder)

  # Extract the arguments from our function call
  set(options, "")
  set(oneValueArgs "NAME")
  set(multiValueArgs
      "HEADER"
      "LIBRARY"
      "PATH_SUFFIX"
      "BINARY"
      "VERSION_FILE"
      "VERSION_BINARY_ARGUMENTS"
      "VERSION_REGEX"
  )
  cmake_parse_arguments(PACKAGE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  # Clear our required_vars variable
  unset(required_vars)

  # Find our include path from our named headers
  if(PACKAGE_HEADER)

    # Find our include path
    find_path(
      "${PACKAGE_NAME}_INCLUDE_DIR"
      NAMES ${PACKAGE_HEADER}
      DOC "The ${PACKAGE_NAME} (${PACKAGE_LIBRARY}) include directory"
      PATH_SUFFIXES ${PACKAGE_PATH_SUFFIX}
    )

    # Setup and export our variables
    set(required_vars ${required_vars} "${PACKAGE_NAME}_INCLUDE_DIR")
    set(${PACKAGE_NAME}_INCLUDE_DIRS
        ${${PACKAGE_NAME}_INCLUDE_DIR}
        PARENT_SCOPE
    )
    mark_as_advanced(${PACKAGE_NAME}_INCLUDE_DIR ${PACKAGE_NAME}_INCLUDE_DIRS)

  endif(PACKAGE_HEADER)

  # Find our library from the named library files
  if(PACKAGE_LIBRARY)
    find_library(
      "${PACKAGE_NAME}_LIBRARY"
      NAMES ${PACKAGE_LIBRARY}
      PATH_SUFFIXES ${PACKAGE_PATH_SUFFIX}
      DOC "The ${PACKAGE_NAME} (${PACKAGE_LIBRARY}) library"
    )

    # Setup and export our variables
    set(required_vars ${required_vars} "${PACKAGE_NAME}_LIBRARY")
    set(${PACKAGE_NAME}_LIBRARIES
        ${${PACKAGE_NAME}_LIBRARY}
        PARENT_SCOPE
    )
    mark_as_advanced(${PACKAGE_NAME}_LIBRARY ${PACKAGE_NAME}_LIBRARIES)

  endif(PACKAGE_LIBRARY)

  # Find our binary from the named binary files
  if(PACKAGE_BINARY)
    find_program(
      "${PACKAGE_NAME}_BINARY"
      NAMES ${PACKAGE_BINARY}
      PATH_SUFFIXES ${PACKAGE_PATH_SUFFIX}
      DOC "The ${PACKAGE_NAME} (${PACKAGE_BINARY}) executable prgram"
    )

    # Setup and export our variables
    set(required_vars ${required_vars} "${PACKAGE_NAME}_BINARY")
    set(${PACKAGE_NAME}_BINARY
        ${${PACKAGE_NAME}_BINARY}
        PARENT_SCOPE
    )
    mark_as_advanced(${PACKAGE_NAME}_BINARY)

  endif(PACKAGE_BINARY)

  # Find our version if we can
  if((PACKAGE_VERSION_FILE AND PACKAGE_HEADER) OR (PACKAGE_VERSION_BINARY_ARGUMENTS AND PACKAGE_BINARY))
    unset(full_version_string)

    # Read our package version file into a variable
    if(PACKAGE_VERSION_FILE AND PACKAGE_HEADER)
      file(READ "${${PACKAGE_NAME}_INCLUDE_DIR}/${PACKAGE_VERSION_FILE}" full_version_string)
    endif(PACKAGE_VERSION_FILE AND PACKAGE_HEADER)

    # Execute our binary to get a version string
    if(PACKAGE_VERSION_BINARY_ARGUMENTS AND PACKAGE_BINARY)
      exec_program(
        ${${PACKAGE_NAME}_BINARY} ARGS ${PACKAGE_VERSION_BINARY_ARGUMENTS} OUTPUT_VARIABLE full_version_string
      )
    endif(PACKAGE_VERSION_BINARY_ARGUMENTS AND PACKAGE_BINARY)

    # Build up our version string
    set(${PACKAGE_NAME}_VERSION "")
    foreach(regex ${PACKAGE_VERSION_REGEX})
      string(REGEX REPLACE ".*${regex}.*" "\\1" regex_output ${full_version_string})
      set(${PACKAGE_NAME}_VERSION ${${PACKAGE_NAME}_VERSION} ${regex_output})
    endforeach(regex)
    string(REPLACE ";" "." ${PACKAGE_NAME}_VERSION "${${PACKAGE_NAME}_VERSION}")

  endif((PACKAGE_VERSION_FILE AND PACKAGE_HEADER) OR (PACKAGE_VERSION_BINARY_ARGUMENTS AND PACKAGE_BINARY))

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(
    ${PACKAGE_NAME}
    FOUND_VAR
    ${PACKAGE_NAME}_FOUND
    REQUIRED_VARS
    ${required_vars}
    VERSION_VAR
    ${PACKAGE_NAME}_VERSION
    # VERSION_VAR "${MAJOR}.${MINOR}.${PATCH}")
  )

  # Export our found variable to parent scope
  set(${PACKAGE_NAME}_FOUND
      ${PACKAGE_NAME}_FOUND
      PARENT_SCOPE
  )

endfunction(ToolchainLibraryFinder)
