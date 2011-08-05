##############################
# wenao.cmake
# 
#   - set TARGET_ROBOT_DIR to Darwin
#   - set CMAKE_MODULES_PATH to ./CMakeModules
#   - set NUBOT_IS_EXECUTABLE
#   - set the OUTPUT_ROOT_DIR_EXE to 
#   - print debug information if desired

INCLUDE(${TARGET_ROBOT_DIR}/cmake/sources.cmake)

############################ CMAKE PACKAGE DIRECTORY
# Set cmakeModules folder
SET( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules )

######### Set NUBOT_EXECUTABLE so that the code is compiled into an executable
SET(NUBOT_IS_EXECUTABLE ON)

SET( OUTPUT_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Build/Darwin/" )

