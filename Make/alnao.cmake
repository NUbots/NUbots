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

############################ ENVIRONMENT VARIABLE AL_DIR
# Check that AL_DIR is set.
# Otherwise, try to set it - first by running findroot.cmake script; else retrieving the AL_DIR environment variable.
IF ( "x${AL_DIR}x" STREQUAL "xx"  )
  IF (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../findroot.cmake  )
        IF( "x$ENV{AL_DIR}x" STREQUAL "xx" )
          MESSAGE( FATAL_ERROR "Please move your project to naoqi/modules/src directory or set the environment variable AL_DIR." )
        ELSE( "x$ENV{AL_DIR}x" STREQUAL "xx" )
          SET( AL_DIR "$ENV{AL_DIR}" )
        ENDIF ( "x$ENV{AL_DIR}x" STREQUAL "xx" )
  ELSE (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../findroot.cmake  )
        INCLUDE ( ../findroot.cmake )
  ENDIF (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../findroot.cmake  )
  MESSAGE( STATUS "Naoqi folder is now set to ${AL_DIR}")
ENDIF("x${AL_DIR}x" STREQUAL "xx" )

############################ CMAKE PACKAGE DIRECTORY
SET( CMAKE_MODULE_PATH ${AL_DIR}/cmakemodules )

############################ ALDEBARAN CMAKE FILE
INCLUDE( "${CMAKE_MODULE_PATH}/aldebaran.cmake" )

############################ ALCOMMON PACKAGE
FIND_PACKAGE( ALCOMMON REQUIRED )
FIND_PACKAGE( TINYXML )

######### Set NUBOT_EXECUTABLE so that the code is compiled into an executable
SET(NUBOT_IS_EXECUTABLE OFF)

############################ Set the output location
SET( OUTPUT_ROOT_DIR_LIB "${AL_DIR}/modules/lib/" )

######### ADD_DEFINITIONS
ADD_DEFINITIONS( ${ALCOMMON_DEFINITIONS} 
                 ${TINYXML_DEFINITIONS}
)

######### INCLUDE_DIRECTORIES
INCLUDE_DIRECTORIES( ${PROXIES_INCLUDE_DIR} 
                     ${ALCOMMON_INCLUDE_DIR}
                     ${TINYXML_INCLUDE_DIR}
)

######### Append required libraries to NUBOT_LINK_LIBRARIES
LIST(APPEND NUBOT_LINK_LIBRARIES ${ALCOMMON_LIBRARIES} ${NEWMAT_LIBRARIES} ${TINYXML_LIBRARIES})


