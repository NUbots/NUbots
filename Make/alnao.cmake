##############################
# alnao.cmake
# 
#   - set TARGET_ROBOT_DIR to NAO
#   - include the NAO specific sources via NAO/cmake/sources.cmake
#   - check and try to set AL_DIR 
#   - set CMAKE_MODULE_PATH to AL_DIR/cmakemodules
#   - include Aldebaran's AL_DIR/cmakemodule/aldebaran.cmake
#   - FIND_PACKAGE(ALCOMMON)
#   - set NUBOT_IS_EXECUTABLE to OFF
#   - set the OUTPUT_ROOT_DIR_LIB to AL_DIR/modules/lib 
#	- ADD_DEFINITIONS
#	- INCLUDE_DIRECTORIES
#	- Append required libraries to NUBOT_LINK_LIBRARIES
#	- print debug information

SET(TARGET_ROBOT_DIR ${TARGET_ROBOT_DIR}/NAO)
INCLUDE(${TARGET_ROBOT_DIR}/cmake/sources.cmake)
INCLUDE(${TOOLCHAIN_DIR}/cmake/bootstrap.cmake)
USE(NAOQI-PLUGINS-TOOLS)

######### Set NUBOT_IS_REMOTE so that the code is compiled into an library
SET(NUBOT_IS_REMOTE OFF)

############################ CMAKE PACKAGE DIRECTORY
SET( CMAKE_MODULE_PATH ${T001CHAIN_DIR}/cmake/modules )

############################ ALDEBARAN CMAKE FILE
INCLUDE( "${CMAKE_MODULE_PATH}/../general.cmake" )

############################ SET CMAKE_CXX_FLAGS (DEFAULT, RELEASE, DEBUG)
# This is IMPORTANT because the default "RELEASE" flags FAIL when targetting the NAO
SET( CMAKE_CXX_FLAGS
  "${CMAKE_CXX_FLAGS} -O2 -Wall" )
# Release build flags
SET( CMAKE_CXX_FLAGS_RELEASE
  "-O3 -DNDEBUG -Wall -march=geode -mtune=geode -mmmx -m3dnow" )
SET( CMAKE_C_FLAGS_RELEASE
  "${CMAKE_CXX_FLAGS_RELEASE}" )
# Debug build flags
SET( CMAKE_CXX_FLAGS_DEBUG
  " -g3 -Wall -march=geode -mtune=geode -mmmx -m3dnow" )

CONFIGURE_SRC_MODULE(nubot ${NUBOT_SRCS})
USE_LIB(nubot ALCOMMON ALMATH LIBCORE TOOLS ALVALUE PROXIES)

SET( OUTPUT_ROOT_DIR_LIB "${CMAKE_CURRENT_SOURCE_DIR}/../Build/NAO/" )


