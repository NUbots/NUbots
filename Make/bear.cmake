##############################
# wenao.cmake
# 
#   - set TARGET_ROBOT_DIR to Bear
#   - include the NAOWebots specific sources via NAOWebots/cmake/sources.cmake
#   - set CMAKE_MODULES_PATH to ./CMakeModules
#   - try and find WEBOTS 
#   - set NUBOT_IS_EXECUTABLE
#   - set the OUTPUT_ROOT_DIR_EXE to 
#         WEBOTS_HOME/projects/contests/nao_robocup/nao_soccer_player_blue
#   - ADD_DEFINITIONS
#   - INCLUDE_DIRECTORIES
#   - Append required libraries to NUBOT_LINK_LIBRARIES
#   - On Darwin we need to make sure that we target i386 and not x86_64, because at this time weboots does not support x86_64
#   - print debug information if desired

INCLUDE(${TARGET_ROBOT_DIR}/cmake/sources.cmake)

############################ CMAKE PACKAGE DIRECTORY
# Set cmakeModules folder
SET( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules )

######### Set NUBOT_EXECUTABLE so that the code is compiled into an executable
SET(NUBOT_IS_EXECUTABLE ON)

######### Try and find where LIBFTD2XX is installed
FIND_PACKAGE(FTD2XX REQUIRED)
ADD_DEFINITIONS(${FTD2XX_DEFINITIONS})
INCLUDE_DIRECTORIES( ${FTD2XX_INCLUDE_DIR})
LIST(APPEND NUBOT_LINK_LIBRARIES ${FTD2XX_LIBRARIES})

SET( OUTPUT_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Build/Bear/" )

