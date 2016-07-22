#GetNUtilities.cmake

# Get source files based on configuration of libraries and project
IF(${PROJECT_NAME} MATCHES NUbots)
	MESSAGE("Including NUbots utilities...")
	FILE(GLOB_RECURSE src "*/**.cpp" "*/**.c" "*/**.h")
ENDIF()

SET(NUTILITIES_SRC_FILES ${src})