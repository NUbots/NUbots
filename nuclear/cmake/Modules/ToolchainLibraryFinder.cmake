include(CMakeParseArguments)
function(ToolchainLibraryFinder)

  # Extract the arguments from our function call
  set(options, "")
  set(oneValueArgs "NAME")
  set(multiValueArgs
      "HEADER"
      "LIBRARY"
      "LIBRARIES"
      "PATH_SUFFIX"
      "BINARY"
      "VERSION_FILE"
      "VERSION_BINARY_ARGUMENTS"
      "VERSION_REGEX"
      "LINK_TYPE"
  )
  cmake_parse_arguments(PACKAGE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  # Clear our required_vars variable
  unset(required_vars)

  if(PACKAGE_LIBRARY OR PACKAGE_LIBRARIES)
    if(PACKAGE_LINK_TYPE)
      set(${PACKAGE_NAME}_LINK_TYPE
          ${PACKAGE_LINK_TYPE}
          CACHE STRING "Choose method to link the library"
      )
    else()
      set(${PACKAGE_NAME}_LINK_TYPE
          UNKNOWN
          CACHE STRING "Choose method to link the library"
      )
    endif()
    set_property(CACHE ${PACKAGE_NAME}_LINK_TYPE PROPERTY STRINGS "SHARED" "STATIC" "MODULE" "UNKNOWN")
    mark_as_advanced(${PACKAGE_NAME}_LINK_TYPE)

    # Search only for specified libraries
    if(${PACKAGE_NAME}_LINK_TYPE STREQUAL "STATIC")
      set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_STATIC_LIBRARY_SUFFIX})
      # Uncache the incorrect value
      if(${PACKAGE_NAME}_LIBRARY MATCHES ".*\.so$")
        unset(${PACKAGE_NAME}_LIBRARY CACHE)
      endif()
    elseif(${PACKAGE_NAME}_LINK_TYPE STREQUAL "SHARED")
      set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_SHARED_LIBRARY_SUFFIX})
      # Uncache the incorrect value
      if(${PACKAGE_NAME}_LIBRARY MATCHES ".*\.a$")
        unset(${PACKAGE_NAME}_LIBRARY CACHE)
      endif()
    endif()
  endif()

  # Find our library from the named library files
  if(PACKAGE_LIBRARY)
    find_library(
      "${PACKAGE_NAME}_LIBRARY"
      NAMES ${PACKAGE_LIBRARY}
      PATH_SUFFIXES ${PACKAGE_PATH_SUFFIX}
      DOC "The ${PACKAGE_NAME} (${PACKAGE_LIBRARY}) library"
    )

    # Setup an imported target for this library
    add_library(${PACKAGE_NAME}::${PACKAGE_NAME} ${${PACKAGE_NAME}_LINK_TYPE} IMPORTED)
    set_target_properties(${PACKAGE_NAME}::${PACKAGE_NAME} PROPERTIES IMPORTED_LOCATION ${${PACKAGE_NAME}_LIBRARY})

    # Setup and export our variables
    list(APPEND required_vars "${PACKAGE_NAME}_LIBRARY")
    set(${PACKAGE_NAME}_LIBRARIES
        ${${PACKAGE_NAME}_LIBRARY}
        PARENT_SCOPE
    )
    mark_as_advanced(${PACKAGE_NAME}_LIBRARY ${PACKAGE_NAME}_LIBRARIES)

  elseif(PACKAGE_LIBRARIES)
    foreach(lib ${PACKAGE_LIBRARIES})
      find_library(
        "${PACKAGE_NAME}_${lib}_LIBRARY"
        NAMES ${lib}
        PATH_SUFFIXES ${PACKAGE_PATH_SUFFIX}
        DOC "The ${PACKAGE_NAME} (${lib}) library"
      )

      # Setup an imported target for this library
      add_library(${PACKAGE_NAME}::${lib} ${${PACKAGE_NAME}_LINK_TYPE} IMPORTED)
      set_target_properties(${PACKAGE_NAME}::${lib} PROPERTIES IMPORTED_LOCATION ${${PACKAGE_NAME}_${lib}_LIBRARY})

      # Setup and export our variables
      set(required_vars ${required_vars} "${PACKAGE_NAME}_${lib}_LIBRARY")
      list(APPEND ${PACKAGE_NAME}_LIBRARIES ${PACKAGE_NAME}::${lib})
      mark_as_advanced(${PACKAGE_NAME}_${lib}_LIBRARY)
    endforeach(lib ${PACKAGE_LIBRARIES})

    # Link all of our imported targets to our imported library
    add_library(${PACKAGE_NAME}::${PACKAGE_NAME} INTERFACE IMPORTED)
    target_link_libraries(${PACKAGE_NAME}::${PACKAGE_NAME} INTERFACE ${${PACKAGE_NAME}_LIBRARIES})

    # Make sure the libraries exist in the parent scope
    set(${PACKAGE_NAME}_LIBRARIES
        ${${PACKAGE_NAME}_LIBRARIES}
        PARENT_SCOPE
    )
    mark_as_advanced(${PACKAGE_NAME}_LIBRARIES)
  endif()

  # Find our include path from our named headers
  if(PACKAGE_HEADER)

    # Find our include path
    find_path(
      "${PACKAGE_NAME}_INCLUDE_DIR"
      NAMES ${PACKAGE_HEADER}
      DOC "The ${PACKAGE_NAME} (${PACKAGE_LIBRARY}) include directory"
      PATH_SUFFIXES ${PACKAGE_PATH_SUFFIX}
    )

    # Add include directories to our imported library
    target_include_directories(${PACKAGE_NAME}::${PACKAGE_NAME} SYSTEM INTERFACE ${${PACKAGE_NAME}_INCLUDE_DIR})

    # Setup and export our variables
    list(APPEND required_vars "${PACKAGE_NAME}_INCLUDE_DIR")
    set(${PACKAGE_NAME}_INCLUDE_DIRS
        ${${PACKAGE_NAME}_INCLUDE_DIR}
        PARENT_SCOPE
    )
    mark_as_advanced(${PACKAGE_NAME}_INCLUDE_DIR ${PACKAGE_NAME}_INCLUDE_DIRS)

  endif()

  # Find our binary from the named binary files
  if(PACKAGE_BINARY)
    find_program(
      "${PACKAGE_NAME}_BINARY"
      NAMES ${PACKAGE_BINARY}
      PATH_SUFFIXES ${PACKAGE_PATH_SUFFIX}
      DOC "The ${PACKAGE_NAME} (${PACKAGE_BINARY}) executable program"
    )

    # Created an imported executable
    add_executable(${PACKAGE_NAME}::${PACKAGE_BINARY} IMPORTED GLOBAL)
    set_target_properties(${PACKAGE_NAME}::${PACKAGE_BINARY} PROPERTIES IMPORTED_LOCATION ${${PACKAGE_NAME}_BINARY})

    # Setup and export our variables
    list(APPEND required_vars "${PACKAGE_NAME}_BINARY")
    set(${PACKAGE_NAME}_BINARY
        ${${PACKAGE_NAME}_BINARY}
        PARENT_SCOPE
    )
    mark_as_advanced(${PACKAGE_NAME}_BINARY)

  endif()

  # Find our version if we can
  if((PACKAGE_VERSION_FILE AND PACKAGE_HEADER) OR (PACKAGE_VERSION_BINARY_ARGUMENTS AND PACKAGE_BINARY))
    unset(full_version_string)

    # Read our package version file into a variable
    if(PACKAGE_VERSION_FILE AND PACKAGE_HEADER)
      file(READ "${${PACKAGE_NAME}_INCLUDE_DIR}/${PACKAGE_VERSION_FILE}" full_version_string)
    endif()

    # Execute our binary to get a version string
    if(PACKAGE_VERSION_BINARY_ARGUMENTS AND PACKAGE_BINARY)
      exec_program(
        ${${PACKAGE_NAME}_BINARY} ARGS
        ${PACKAGE_VERSION_BINARY_ARGUMENTS}
        OUTPUT_VARIABLE full_version_string
      )
    endif()

    # Build up our version string
    set(${PACKAGE_NAME}_VERSION "")
    foreach(regex ${PACKAGE_VERSION_REGEX})
      string(REGEX REPLACE ".*${regex}.*" "\\1" regex_output ${full_version_string})
      set(${PACKAGE_NAME}_VERSION ${${PACKAGE_NAME}_VERSION} ${regex_output})
    endforeach(regex)
    string(REPLACE ";" "." ${PACKAGE_NAME}_VERSION "${${PACKAGE_NAME}_VERSION}")

  endif()

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(
    ${PACKAGE_NAME}
    FOUND_VAR ${PACKAGE_NAME}_FOUND
    REQUIRED_VARS ${required_vars}
    VERSION_VAR ${PACKAGE_NAME}_VERSION # VERSION_VAR "${MAJOR}.${MINOR}.${PATCH}")
  )

  # Export our found variable to parent scope
  set(${PACKAGE_NAME}_FOUND
      ${PACKAGE_NAME}_FOUND
      PARENT_SCOPE
  )

endfunction(ToolchainLibraryFinder)
