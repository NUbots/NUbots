include(CMakeParseArguments)
function(HeaderLibrary)
  # Extract the arguments from our function call
  set(options, "")
  set(oneValueArgs "NAME")
  set(multiValueArgs "HEADER" "PATH_SUFFIX" "URL")
  cmake_parse_arguments(PACKAGE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  # Clear our required_vars variable
  unset(required_vars)

  # Find our include path
  find_path(
    "${PACKAGE_NAME}_INCLUDE_DIR"
    NAMES ${PACKAGE_HEADER}
    DOC "The ${PACKAGE_NAME} include directory"
    PATHS "${CMAKE_BINARY_DIR}/include"
    PATH_SUFFIXES ${PACKAGE_PATH_SUFFIX}
  )

  # File doesn't exist in standard search paths, download it
  if(NOT ${PACKAGE_NAME}_INCLUDE_DIR)
    set(OUTPUT_DIR "${CMAKE_BINARY_DIR}/include")

    # Create the output folder if it doesn't already exist
    if(NOT EXISTS "${OUTPUT_DIR}")
      file(MAKE_DIRECTORY "${OUTPUT_DIR}")
    endif()

    # Download file.
    file(DOWNLOAD "${PACKAGE_URL}" "${OUTPUT_DIR}/${PACKAGE_HEADER}" STATUS ${PACKAGE_NAME}_STATUS)

    list(GET ${PACKAGE_NAME}_STATUS 0 ${PACKAGE_NAME}_STATUS_CODE)

    # Parse download status
    if(${PACKAGE_NAME}_STATUS_CODE EQUAL 0)
      message(STATUS "Successfully downloaded ${PACKAGE_NAME} library.")

      set(${PACKAGE_NAME}_INCLUDE_DIR "${OUTPUT_DIR}")

    else()
      message(ERROR "Failed to download ${PACKAGE_NAME} library.")
    endif()
  endif()

  # Setup and export our variables
  set(required_vars ${required_vars} "${PACKAGE_NAME}_INCLUDE_DIR")
  set(${PACKAGE_NAME}_INCLUDE_DIRS ${${PACKAGE_NAME}_INCLUDE_DIR} PARENT_SCOPE)
  mark_as_advanced(${PACKAGE_NAME}_INCLUDE_DIR ${PACKAGE_NAME}_INCLUDE_DIRS)

  # Find the package
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(
    ${PACKAGE_NAME}
    FOUND_VAR
    ${PACKAGE_NAME}_FOUND
    REQUIRED_VARS
    ${required_vars}
    VERSION_VAR
    ${PACKAGE_NAME}_VERSION
  )

endfunction(HeaderLibrary)
