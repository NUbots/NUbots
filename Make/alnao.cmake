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
#   - set the OUTPUT_ROOT_DIR to AL_DIR/modules/lib 
#	- ADD_DEFINITIONS
#	- INCLUDE_DIRECTORIES
#	- Append required libraries to NUBOT_LINK_LIBRARIES
#	- print debug information

INCLUDE(${TARGET_ROBOT_DIR}/cmake/sources.cmake)
INCLUDE(${TOOLCHAIN_DIR}/cmake/bootstrap.cmake)
USE(NAOQI-PLUGINS-TOOLS)

######### Set NUBOT_IS_REMOTE so that the code is compiled into an library
SET(NUBOT_IS_EXECUTABLE OFF)
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

SET( OUTPUT_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Build/NAO/" )

MARK_AS_ADVANCED(
	ALCOMMON_DEPENDS
	ALCOMMON_tempdebug
	ALMATH_DEPENDS
	ALMATH_tempdebug
	ALVALUE_DEPENDS
	ALVALUE_tempdebug
	ASCIIDOC_EXECUTABLE
    BUILD_SHARED_LIBS
    CCACHE
	DOXYGEN_DIR
	LIBCORE_DEPENDS
	LIBCORE_tempdebug
	LIBFACTORY_DEPENDS
	LIBFACTORY_tempdebug
    LIBFINDIPPC_DEPENDS
	LIBFINDIPPC_tempdebug
	LIBIPPC_DEPENDS
	LIBIPPC_tempdebug
	LIBSOAP_DEPENDS
	LIBSOAP_tempdebug
	LIBTHREAD_DEPENDS
	LIBTHREAD_tempdebug
	NAOQI-PLUGINS-TOOLS_DIR
	PROXIES_DEPENDS
	RTTOOLS_DEPENDS
	RTTOOLS_tempdebug
	T001CHAIN_DIR
	TINYXML_DEPENDS
	TINYXML_tempdebug
	TOOLCHAIN_DIR
	TOOLS_DEPENDS
	TOOLS_tempdebug
	WIN_LAYOUT_SDK_BIN
	WIN_LAYOUT_SDK_CMAKE
	WIN_LAYOUT_SDK_CMAKE_MODULES
	WIN_LAYOUT_SDK_CONF
	WIN_LAYOUT_SDK_DOC
	WIN_LAYOUT_SDK_INCLUDE
	WIN_LAYOUT_SDK_LIB
	WIN_LAYOUT_SDK_SHARE
	_SDK_FRAMEWORK
)


