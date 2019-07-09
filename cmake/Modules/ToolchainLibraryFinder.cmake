INCLUDE(CMakeParseArguments)
FUNCTION(ToolchainLibraryFinder)

    # Extract the arguments from our function call
    SET(options, "")
    SET(oneValueArgs "NAME")
    SET(multiValueArgs "HEADER" "LIBRARY" "PATH_SUFFIX" "BINARY" "VERSION_FILE" "VERSION_BINARY_ARGUMENTS" "VERSION_REGEX")
    CMAKE_PARSE_ARGUMENTS(PACKAGE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    # Clear our required_vars variable
    UNSET(required_vars)

    # Find our include path from our named headers
    IF(PACKAGE_HEADER)

        # Find our include path
        FIND_PATH("${PACKAGE_NAME}_INCLUDE_DIR"
                  NAMES ${PACKAGE_HEADER}
                  DOC "The ${PACKAGE_NAME} (${PACKAGE_LIBRARY}) include directory"
                  PATH_SUFFIXES ${PACKAGE_PATH_SUFFIX}
        )

        # Setup and export our variables
        SET(required_vars ${required_vars} "${PACKAGE_NAME}_INCLUDE_DIR")
        SET(${PACKAGE_NAME}_INCLUDE_DIRS ${${PACKAGE_NAME}_INCLUDE_DIR} PARENT_SCOPE)
        MARK_AS_ADVANCED(${PACKAGE_NAME}_INCLUDE_DIR ${PACKAGE_NAME}_INCLUDE_DIRS)

    ENDIF(PACKAGE_HEADER)

    # Find our library from the named library files
    IF(PACKAGE_LIBRARY)
        FIND_LIBRARY("${PACKAGE_NAME}_LIBRARY"
                     NAMES ${PACKAGE_LIBRARY}
                     PATH_SUFFIXES ${PACKAGE_PATH_SUFFIX}
                     DOC "The ${PACKAGE_NAME} (${PACKAGE_LIBRARY}) library"
        )

        # Setup and export our variables
        SET(required_vars ${required_vars} "${PACKAGE_NAME}_LIBRARY")
        SET(${PACKAGE_NAME}_LIBRARIES ${${PACKAGE_NAME}_LIBRARY} PARENT_SCOPE)
        MARK_AS_ADVANCED(${PACKAGE_NAME}_LIBRARY ${PACKAGE_NAME}_LIBRARIES)

    ENDIF(PACKAGE_LIBRARY)

    # Find our binary from the named binary files
    IF(PACKAGE_BINARY)
        FIND_PROGRAM("${PACKAGE_NAME}_BINARY"
                     NAMES ${PACKAGE_BINARY}
                     PATH_SUFFIXES ${PACKAGE_PATH_SUFFIX}
                     DOC "The ${PACKAGE_NAME} (${PACKAGE_BINARY}) executable prgram"
        )

        # Setup and export our variables
        SET(required_vars ${required_vars} "${PACKAGE_NAME}_BINARY")
        SET(${PACKAGE_NAME}_BINARY ${${PACKAGE_NAME}_BINARY} PARENT_SCOPE)
        MARK_AS_ADVANCED(${PACKAGE_NAME}_BINARY)

    ENDIF(PACKAGE_BINARY)

    # Find our version if we can
    IF((PACKAGE_VERSION_FILE AND PACKAGE_HEADER) OR (PACKAGE_VERSION_BINARY_ARGUMENTS AND PACKAGE_BINARY))
        UNSET(full_version_string)

        # Read our package version file into a variable
        IF(PACKAGE_VERSION_FILE AND PACKAGE_HEADER)
            FILE(READ "${${PACKAGE_NAME}_INCLUDE_DIR}/${PACKAGE_VERSION_FILE}" full_version_string)
        ENDIF(PACKAGE_VERSION_FILE AND PACKAGE_HEADER)

        # Execute our binary to get a version string
        IF(PACKAGE_VERSION_BINARY_ARGUMENTS AND PACKAGE_BINARY)
            EXEC_PROGRAM(${${PACKAGE_NAME}_BINARY}
                         ARGS ${PACKAGE_VERSION_BINARY_ARGUMENTS}
                         OUTPUT_VARIABLE full_version_string)
        ENDIF(PACKAGE_VERSION_BINARY_ARGUMENTS AND PACKAGE_BINARY)

        # Build up our version string
        SET(${PACKAGE_NAME}_VERSION "")
        FOREACH(regex ${PACKAGE_VERSION_REGEX})
            STRING(REGEX REPLACE ".*${regex}.*" "\\1" regex_output ${full_version_string})
            SET(${PACKAGE_NAME}_VERSION ${${PACKAGE_NAME}_VERSION} ${regex_output})
        ENDFOREACH(regex)
        STRING(REPLACE ";" "." ${PACKAGE_NAME}_VERSION "${${PACKAGE_NAME}_VERSION}")

    ENDIF((PACKAGE_VERSION_FILE AND PACKAGE_HEADER) OR (PACKAGE_VERSION_BINARY_ARGUMENTS AND PACKAGE_BINARY))

    INCLUDE(FindPackageHandleStandardArgs)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(${PACKAGE_NAME}
                                      FOUND_VAR ${PACKAGE_NAME}_FOUND
                                      REQUIRED_VARS ${required_vars}
                                      VERSION_VAR ${PACKAGE_NAME}_VERSION
                                      #VERSION_VAR "${MAJOR}.${MINOR}.${PATCH}")
    )

    # Export our found variable to parent scope
    SET(${PACKAGE_NAME}_FOUND ${PACKAGE_NAME}_FOUND PARENT_SCOPE)

ENDFUNCTION(ToolchainLibraryFinder)
