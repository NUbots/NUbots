##############################
#
# Welcome to the top-level cmake file
#
# This is the main cmake configuration file 
# You can define specific compilation options here
# 
# More about cmake : www.cmake.org
#
# Using cmake command line : 
# 1. Create a build directory : mkdir build
# 2. Configure cmake project : cd build; cmake ..
#   You can also generate project with your favorite IDE : 
#   Example : Under Mac osx use "cmake .. -G XCode"
#   see cmake help for more information
# 3. Edit project option : ccmake .
# 4. Build it : make 



############################ PROJECT NAME
PROJECT( NUBOT )
MESSAGE( STATUS "...:::: NUBOT ::::..." )

########### Target Robot: NAOWebots, NAO, Cycloid, etc
SET(TARGET_ROBOT $ENV{TARGET_ROBOT})
SET(NUBOT_IS_EXECUTABLE OFF)
SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)

########## Control Debugging of the cmake files themselves
SET(DEBUG ON)
IF(DEBUG)
    MESSAGE(STATUS "Debugging cmake process")
ENDIF (DEBUG)

########## The location of target specific files 
# this is used by target specific cmake files to locate appropriate sources
SET(TARGET_ROBOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../NUPlatform/Platforms)

INCLUDE(sources.cmake)

########## Selectively include target specific CMake files
IF (${TARGET_ROBOT} STREQUAL NAOWEBOTS)
    MESSAGE(STATUS "CMake for Nao in Webots")
    INCLUDE(${CMAKE_CURRENT_SOURCE_DIR}/wenao.cmake)
ELSE (${TARGET_ROBOT} STREQUAL NAOWEBOTS)
    IF (${TARGET_ROBOT} STREQUAL NAO)
        MESSAGE(STATUS "CMake for NAO")
        INCLUDE(${CMAKE_CURRENT_SOURCE_DIR}/alnao.cmake)
    ELSE (${TARGET_ROBOT} STREQUAL NAO)
        IF (${TARGET_ROBOT} STREQUAL CYCLOID)
            MESSAGE(STATUS "CMake for Cycloid")
            INCLUDE(${CMAKE_CURRENT_SOURCE_DIR}/cycloid.cmake)
        ELSE (${TARGET_ROBOT} STREQUAL CYCLOID)
            MESSAGE(STATUS "Target robot unknown: ${TARGET_ROBOT}")
        ENDIF (${TARGET_ROBOT} STREQUAL CYCLOID)
    ENDIF (${TARGET_ROBOT} STREQUAL NAO)
ENDIF (${TARGET_ROBOT} STREQUAL NAOWEBOTS)

IF (DEBUG)
MESSAGE(STATUS "Target Robot location: ${TARGET_ROBOT_DIR}")
ENDIF (DEBUG)

IF (WIN32)
SET(CMAKE_DEBUG_POSTFIX _d)
ENDIF (WIN32)

########### Version
# Check cMake version
CMAKE_MINIMUM_REQUIRED( VERSION 2.6.4 )

########### Colourize makefile
# Colorized output
SET( CMAKE_COLOR_MAKEFILE TRUE )


############################ FIND PACKAGES

FIND_PACKAGE( PTHREAD REQUIRED )
FIND_PACKAGE( BOOST REQUIRED )
FIND_PACKAGE( RT )

############################ targetconfig.h GENERATION
CONFIGURE_FILE(
	"${CMAKE_CURRENT_SOURCE_DIR}/targetconfig.in"
  	"${CMAKE_CURRENT_SOURCE_DIR}/../Autoconfig/targetconfig.h"
    ESCAPE_QUOTES
)

############################ debug.h GENERATION
CONFIGURE_FILE(
	"${CMAKE_CURRENT_SOURCE_DIR}/debug.in"
  	"${CMAKE_CURRENT_SOURCE_DIR}/../Autoconfig/debug.h"
    ESCAPE_QUOTES
)

# Some people use BOOST and others use Boost. I need to be able to handle both. So if someone's using CAPS
# copy into to the non caps (standard version)
IF (NOT "${BOOST_INCLUDE_DIR}" STREQUAL "")
	SET(Boost_DEFINITIONS ${BOOST_DEFINITIONS})
	SET(Boost_INCLUDE_DIR ${BOOST_INCLUDE_DIR})
	SET(Boost_LIBRARIES ${BOOST_LIBRARIES})
ENDIF()

############################ DEFINITION
# Add definitions needed here
ADD_DEFINITIONS( ${PTHREAD_DEFINITIONS}
                 ${Boost_DEFINITIONS}
		 ${LIBRT_DEFINITIONS}
)


############################ INCLUDE DIRECTORY
# Define include directories here
INCLUDE_DIRECTORIES( ${PTHREAD_INCLUDE_DIR}
                     ${Boost_INCLUDE_DIR}
		     ${LIBRT_INCLUDE_DIR}
)

IF (DEBUG)
    MESSAGE(STATUS "Source files:")
    FOREACH(loop_var ${NUBOT_SRCS})
        MESSAGE(STATUS "------- ${loop_var}")
    ENDFOREACH(loop_var ${NUBOT_SRCS})
ENDIF (DEBUG)  

############################ CHECK LIBRARY / EXECUTABLE OPTION
IF (NUBOT_IS_REMOTE)
    ADD_EXECUTABLE( nubot ${NUBOT_SRCS} )
ELSE (NUBOT_IS_REMOTE)
    IF (NOT ${TARGET_ROBOT} STREQUAL NAO)
        ADD_LIBRARY( nubot SHARED ${NUBOT_SRCS} )
    ENDIF (NOT ${TARGET_ROBOT} STREQUAL NAO)
ENDIF (NUBOT_IS_REMOTE)

############################ SET LIBRARIES TO LINK WITH
TARGET_LINK_LIBRARIES( nubot 
                       ${NUBOT_LINK_LIBRARIES} 
                       ${PTHREAD_LIBRARIES}
                       ${Boost_LIBRARIES}
		       ${LIBRT_LIBRARIES}
)

############################ BINARY COPY
# copy binaries in the output folder automatically
GET_TARGET_PROPERTY( NUBOT_LOCATION nubot LOCATION)

IF ( WIN32 )
    STRING(REGEX REPLACE "nubot.dll" "" NUBOT_LOCATION_SRC ${NUBOT_LOCATION})
    FILE(TO_NATIVE_PATH ${NUBOT_LOCATION_SRC} NUBOT_LOCATION_SRC)
    FILE(TO_NATIVE_PATH ${OUTPUT_ROOT_DIR_LIB} OUTPUT_ROOT_DIR_LIB)
ENDIF ( WIN32 )

IF (WIN32)
    ADD_CUSTOM_COMMAND(
    TARGET nubot
    POST_BUILD
    COMMAND xcopy  ${NUBOT_LOCATION_SRC}*.dll ${OUTPUT_ROOT_DIR_LIB} /Y /C
    )
ELSE (WIN32)
    ADD_CUSTOM_COMMAND(
    TARGET nubot
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${NUBOT_LOCATION} ${OUTPUT_ROOT_DIR_LIB}
    )
ENDIF (WIN32)

############################ OPTIONS
SET(DEBUG_NUBOT_VERBOSITY 5 CACHE STRING "Set the verbosity of debug information of the nubot (0 to 5)")
SET(DEBUG_NUPLATFORM_VERBOSITY 5 CACHE STRING "Set the verbosity of debug information of the nubot (0 to 5)")
SET(DEBUG_NUCAMERA_VERBOSITY 5 CACHE STRING "Set the verbosity of debug information of the nubot (0 to 5)")
SET(DEBUG_NUSENSORS_VERBOSITY 5 CACHE STRING "Set the verbosity of debug information of the nubot (0 to 5)")
SET(DEBUG_NUACTIONATORS_VERBOSITY 5 CACHE STRING "Set the verbosity of debug information of the nubot (0 to 5)")
SET(DEBUG_NUSYSTEM_VERBOSITY 5 CACHE STRING "Set the verbosity of debug information of the nubot (0 to 5)")
SET(DEBUG_VISION_VERBOSITY 5 CACHE STRING "Set the verbosity of debug information of the vision (0 to 5)")
SET(DEBUG_LOCALISATION_VERBOSITY 5 CACHE STRING "Set the verbosity of debug information of the localisation (0 to 5)")
SET(DEBUG_BEHAVIOUR_VERBOSITY 5 CACHE STRING "Set the verbosity of debug information of the behaviour (0 to 5)")
SET(DEBUG_NUMOTION_VERBOSITY 5 CACHE STRING "Set the verbosity of debug information of the motion (0 to 5)")

SET(THREAD_VISION_PRIORITY 0 CACHE STRING "Set the priority of the vision thread (0 to 100)")
SET(THREAD_MOTION_PRIORITY 40 CACHE STRING "Set the priority of the motion thread (0 to 100)")

OPTION(
        THREAD_VISION_MONITOR_TIME
        "Set to ON to monitor the computation time of the vision thread"
        ON
)
OPTION(
        THREAD_MOTION_MONITOR_TIME
        "Set to ON to monitor the computation time of the motion thread"
        ON
)

OPTION(
        USE_VISION
        "Set to ON to run vision, set to OFF to run blind"
        ON
)
OPTION(
        USE_LOCALISATION
        "Set to ON to run localisation, set to OFF be be lost"
        ON
)
OPTION(
        USE_BEHAVIOUR
        "Set to ON to run behaviour, set to OFF to have a lobotomy"
        ON
)
OPTION(
        USE_MOTION
        "Set to ON to run motion, set to OFF to be paralysed"
        ON
)
