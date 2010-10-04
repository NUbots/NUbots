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



######################################################## PROJECT NAME ########################################################
PROJECT( NUBOT )
MESSAGE( STATUS "...:::: NUBOT ::::..." )

###################### Target Robot: NAOWebots, NAO, Cycloid, Bear, etc
IF(x$ENV{TARGET_ROBOT}x STREQUAL xx)
	MESSAGE(STATUS "TARGET_ROBOT was not found in the environment. Assuming NAOWEBOTS")
	SET(TARGET_ROBOT NAOWEBOTS)
ELSE()
	SET(TARGET_ROBOT $ENV{TARGET_ROBOT})
ENDIF()

SET(NUBOT_IS_EXECUTABLE OFF)
SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)

########## Control Debugging of the cmake files themselves
SET(DEBUG ON)
IF(DEBUG)
    MESSAGE(STATUS "Debugging cmake process")
ENDIF (DEBUG)

######################################################## The location of target specific files 
# this is used by target specific cmake files to locate appropriate sources
#	- we have TARGET_ROBOT_NAME for a case sensitive name of the platform.
# 	- we have target specific sources in TARGET_ROBOT_DIR
# 	- we have target specific home environment variable in HOME_ENV_VAR
# 	- we have target specific location for data files NUBOT_DATA_DIR. This is the location on the actual platform that data files are placed.
IF(${TARGET_ROBOT} STREQUAL NAOWEBOTS)
    SET(TARGET_ROBOT_NAME NAOWebots)
ELSEIF(${TARGET_ROBOT} STREQUAL NAO)
    SET(TARGET_ROBOT_NAME NAO)
ELSEIF(${TARGET_ROBOT} STREQUAL CYCLOID)
    SET(TARGET_ROBOT_NAME Cycloid)
ELSEIF(${TARGET_ROBOT} STREQUAL BEAR)
    SET(TARGET_ROBOT_NAME Bear)
ELSEIF(${TARGET_ROBOT} STREQUAL NUVIEW)
    SET(TARGET_ROBOT_NAME NUview)
ENDIF()

IF(${TARGET_ROBOT} STREQUAL NUVIEW)
    SET(TARGET_ROBOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../${TARGET_ROBOT_NAME})
ELSE()
    SET(TARGET_ROBOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../NUPlatform/Platforms/${TARGET_ROBOT_NAME})
ENDIF()

SET(NUBOT_CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../Config/${TARGET_ROBOT_NAME} CACHE STRING "Directory for nubot configuration files, i.e. where the Config directory for the target platform is on your computer")

SET(HOME_ENV_VAR "HOME") 
IF(${TARGET_ROBOT} STREQUAL NAOWEBOTS OR ${TARGET_ROBOT} STREQUAL NUVIEW)
    IF(${CMAKE_SYSTEM_NAME} STREQUAL Windows)
	SET(HOME_ENV_VAR "HOMEPATH") 
    ENDIF()
ENDIF()

IF (DEBUG)
    MESSAGE(STATUS "Home directory: $ENV{${HOME_ENV_VAR}}")
ENDIF()

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
            IF (${TARGET_ROBOT} STREQUAL BEAR)
                MESSAGE(STATUS "Cmake for Bear")
                INCLUDE(${CMAKE_CURRENT_SOURCE_DIR}/bear.cmake)
            ELSE()
                MESSAGE(STATUS "Target robot unknown: ${TARGET_ROBOT}")
            ENDIF()
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
CONFIGURE_FILE( "${CMAKE_CURRENT_SOURCE_DIR}/targetconfig.in"
                "${CMAKE_CURRENT_SOURCE_DIR}/../Autoconfig/targetconfig.h"
                ESCAPE_QUOTES)
CONFIGURE_FILE( "${CMAKE_CURRENT_SOURCE_DIR}/nubotconfig.in"
                "${CMAKE_CURRENT_SOURCE_DIR}/../Autoconfig/nubotconfig.h"
                ESCAPE_QUOTES)
CONFIGURE_FILE( "${CMAKE_CURRENT_SOURCE_DIR}/nubotdataconfig.in"
                "${CMAKE_CURRENT_SOURCE_DIR}/../Autoconfig/nubotdataconfig.h"
                ESCAPE_QUOTES)

############################ debug.h GENERATION
CONFIGURE_FILE(	"${CMAKE_CURRENT_SOURCE_DIR}/debug.in"
                "${CMAKE_CURRENT_SOURCE_DIR}/../Autoconfig/debug.h"
                ESCAPE_QUOTES)
CONFIGURE_FILE( "${CMAKE_CURRENT_SOURCE_DIR}/debugverbositybehaviour.in"
                "${CMAKE_CURRENT_SOURCE_DIR}/../Autoconfig/debugverbositybehaviour.h"
                ESCAPE_QUOTES)
CONFIGURE_FILE( "${CMAKE_CURRENT_SOURCE_DIR}/debugverbosityjobs.in"
                "${CMAKE_CURRENT_SOURCE_DIR}/../Autoconfig/debugverbosityjobs.h"
                ESCAPE_QUOTES)
CONFIGURE_FILE( "${CMAKE_CURRENT_SOURCE_DIR}/debugverbositylocalisation.in"
                "${CMAKE_CURRENT_SOURCE_DIR}/../Autoconfig/debugverbositylocalisation.h"
                ESCAPE_QUOTES)
CONFIGURE_FILE( "${CMAKE_CURRENT_SOURCE_DIR}/debugverbositynetwork.in"
                "${CMAKE_CURRENT_SOURCE_DIR}/../Autoconfig/debugverbositynetwork.h"
                ESCAPE_QUOTES)
CONFIGURE_FILE( "${CMAKE_CURRENT_SOURCE_DIR}/debugverbositynuactionators.in"
                "${CMAKE_CURRENT_SOURCE_DIR}/../Autoconfig/debugverbositynuactionators.h"
                ESCAPE_QUOTES)
CONFIGURE_FILE( "${CMAKE_CURRENT_SOURCE_DIR}/debugverbositynubot.in"
                "${CMAKE_CURRENT_SOURCE_DIR}/../Autoconfig/debugverbositynubot.h"
                ESCAPE_QUOTES)
CONFIGURE_FILE( "${CMAKE_CURRENT_SOURCE_DIR}/debugverbositynucamera.in"
                "${CMAKE_CURRENT_SOURCE_DIR}/../Autoconfig/debugverbositynucamera.h"
                ESCAPE_QUOTES)
CONFIGURE_FILE( "${CMAKE_CURRENT_SOURCE_DIR}/debugverbositynumotion.in"
                "${CMAKE_CURRENT_SOURCE_DIR}/../Autoconfig/debugverbositynumotion.h"
                ESCAPE_QUOTES)
CONFIGURE_FILE( "${CMAKE_CURRENT_SOURCE_DIR}/debugverbositynuplatform.in"
                "${CMAKE_CURRENT_SOURCE_DIR}/../Autoconfig/debugverbositynuplatform.h"
                ESCAPE_QUOTES)
CONFIGURE_FILE( "${CMAKE_CURRENT_SOURCE_DIR}/debugverbositynusensors.in"
                "${CMAKE_CURRENT_SOURCE_DIR}/../Autoconfig/debugverbositynusensors.h"
                ESCAPE_QUOTES)
CONFIGURE_FILE( "${CMAKE_CURRENT_SOURCE_DIR}/debugverbositynusystem.in"
                "${CMAKE_CURRENT_SOURCE_DIR}/../Autoconfig/debugverbositynusystem.h"
                ESCAPE_QUOTES)
CONFIGURE_FILE( "${CMAKE_CURRENT_SOURCE_DIR}/debugverbositythreading.in"
                "${CMAKE_CURRENT_SOURCE_DIR}/../Autoconfig/debugverbositythreading.h"
                ESCAPE_QUOTES)
CONFIGURE_FILE( "${CMAKE_CURRENT_SOURCE_DIR}/debugverbosityvision.in"
                "${CMAKE_CURRENT_SOURCE_DIR}/../Autoconfig/debugverbosityvision.h"
                ESCAPE_QUOTES)
                

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
IF (NUBOT_IS_EXECUTABLE)
    ADD_EXECUTABLE( nubot ${NUBOT_SRCS} )
ELSE()
    IF (NOT ${TARGET_ROBOT} STREQUAL NAO)
        ADD_LIBRARY( nubot SHARED ${NUBOT_SRCS} )
    ENDIF (NOT ${TARGET_ROBOT} STREQUAL NAO)
ENDIF()

############################ SET LIBRARIES TO LINK WITH
IF (${CMAKE_SYSTEM_NAME} STREQUAL Windows)
	LIST(APPEND NUBOT_LINK_LIBRARIES ws2_32)
ENDIF()

TARGET_LINK_LIBRARIES( nubot 
                       ${NUBOT_LINK_LIBRARIES} 
                       ${PTHREAD_LIBRARIES}
                       ${Boost_LIBRARIES}
                       ${LIBRT_LIBRARIES}
)

############################ BINARY COPY
# copy binaries in the output folder automatically
GET_TARGET_PROPERTY( NUBOT_LOCATION nubot LOCATION)
IF (${TARGET_ROBOT} STREQUAL NAOWEBOTS)
    ADD_CUSTOM_COMMAND(TARGET nubot
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${NUBOT_LOCATION} ${OUTPUT_ROOT_DIR_1})
    ADD_CUSTOM_COMMAND(TARGET nubot
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/../Config $ENV{${HOME_ENV_VAR}}/nubot/Config)
ELSE()
    ADD_CUSTOM_COMMAND(TARGET nubot
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${NUBOT_LOCATION} ${OUTPUT_ROOT_DIR})
ENDIF()

############################ OPTIONS
SET(NUBOT_DEBUG_NUBOT_VERBOSITY 0 CACHE STRING "Set the verbosity of debug information of the nubot (0 to 5)")
SET(NUBOT_DEBUG_THREADING_VERBOSITY 0 CACHE STRING "Set the verbosity of debug information of the threading (0 to 5)")
SET(NUBOT_DEBUG_NUPLATFORM_VERBOSITY 0 CACHE STRING "Set the verbosity of debug information of the nubot (0 to 5)")
SET(NUBOT_DEBUG_NUCAMERA_VERBOSITY 0 CACHE STRING "Set the verbosity of debug information of the nubot (0 to 5)")
SET(NUBOT_DEBUG_NUSENSORS_VERBOSITY 0 CACHE STRING "Set the verbosity of debug information of the nubot (0 to 5)")
SET(NUBOT_DEBUG_NUACTIONATORS_VERBOSITY 0 CACHE STRING "Set the verbosity of debug information of the nubot (0 to 5)")
SET(NUBOT_DEBUG_NUSYSTEM_VERBOSITY 0 CACHE STRING "Set the verbosity of debug information of the nubot (0 to 5)")
SET(NUBOT_DEBUG_VISION_VERBOSITY 0 CACHE STRING "Set the verbosity of debug information of the vision (0 to 5)")
SET(NUBOT_DEBUG_LOCALISATION_VERBOSITY 0 CACHE STRING "Set the verbosity of debug information of the localisation (0 to 5)")
SET(NUBOT_DEBUG_BEHAVIOUR_VERBOSITY 0 CACHE STRING "Set the verbosity of debug information of the behaviour (0 to 5)")
SET(NUBOT_DEBUG_JOBS_VERBOSITY 0 CACHE STRING "Set the verbosity of debug information of the behaviour (0 to 5)")
SET(NUBOT_DEBUG_NUMOTION_VERBOSITY 0 CACHE STRING "Set the verbosity of debug information of the motion (0 to 5)")
SET(NUBOT_DEBUG_NETWORK_VERBOSITY 0 CACHE STRING "Set the verbosity of debug information of the motion (0 to 5)")

OPTION( NUBOT_USE_VISION
        "Set to ON to run vision, set to OFF to run blind"
        ON)
OPTION( NUBOT_USE_LOCALISATION
        "Set to ON to run localisation, set to OFF to be lost"
        ON)
OPTION( NUBOT_USE_BEHAVIOUR
        "Set to ON to run behaviour, set to OFF to have a lobotomy"
        ON)
OPTION( NUBOT_USE_MOTION
        "Set to ON to run motion, set to OFF to be paralysed"
        ON)
OPTION(	NUBOT_USE_NETWORK
        "Set to ON to run motion, set to OFF to be paralysed"
        ON)

############################ NUbot.cpp Threading Options
SET(NUBOT_THREAD_SEETHINK_PRIORITY 0 CACHE STRING "Set the priority of the see-think thread (0 to 100)")
SET(NUBOT_THREAD_SENSEMOVE_PRIORITY 40 CACHE STRING "Set the priority of the sense-move thread (0 to 100)")

OPTION( NUBOT_THREAD_SEETHINK_PROFILER
        "Set to ON to monitor the computation time of the vision thread"
        OFF)
OPTION( NUBOT_THREAD_SENSEMOVE_PROFILER
        "Set to ON to monitor the computation time of the motion thread"
        OFF)

MARK_AS_ADVANCED(
	NUBOT_THREAD_SEETHINK_PRIORITY
	NUBOT_THREAD_SENSEMOVE_PRIORITY
	NUBOT_THREAD_SEETHINK_PROFILER
	NUBOT_THREAD_SENSEMOVE_PROFILER
)
