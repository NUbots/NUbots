# GetNUtilities.cmake
message("** Configuring NUtilities...")

set(NUTILITIES_SRC_FILES "")

# Get source files based on configuration of libraries and project
if(${PROJECT_NAME} MATCHES NUbots)
  message("** Using NUbots utilities...")
  file(GLOB_RECURSE NUTILITIES_SRC_FILES "${NUTILITIES_DIR}/*/**.cpp" "${NUTILITIES_DIR}/*/**.c"
       "${NUTILITIES_DIR}/*/**.h"
  )
endif()

message(CURRENT_DIRECTORY ${CURRENT_DIRECTORY})
