#GetNUtilities.cmake
MESSAGE("** Configuring NUtilities...")

SET(NUTILITIES_SRC_FILES "")

# Get source files based on configuration of libraries and project
IF(${PROJECT_NAME} MATCHES NUbots)
	MESSAGE("** Using NUbots utilities...")
	FILE(GLOB_RECURSE NUTILITIES_SRC_FILES 
		"${NUTILITIES_DIR}/*/**.cpp" 
		"${NUTILITIES_DIR}/*/**.c" 
		"${NUTILITIES_DIR}/*/**.h")
ENDIF()

MESSAGE(CURRENT_DIRECTORY ${CURRENT_DIRECTORY})

