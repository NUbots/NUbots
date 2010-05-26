##############################
# wenao.cmake
# 
#   - set TARGET_ROBOT_DIR to NAOWebots
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

######### Try and find where WEBOTS is installed
FIND_PACKAGE(WEBOTS REQUIRED)

######### Set NUBOT_EXECUTABLE so that the code is compiled into an executable
SET(NUBOT_IS_REMOTE ON)

######### Set the output path so it goes nicely into the webots file structure
IF (${CMAKE_SYSTEM_NAME} STREQUAL Windows)
    SET( OUTPUT_ROOT_DIR_0 "${WEBOTS_DIR}/projects/contests/robotstadium/controllers/nao_team_0/nao_team_0.exe" )
    SET( OUTPUT_ROOT_DIR_1 "${WEBOTS_DIR}/projects/contests/robotstadium/controllers/nao_team_1/nao_team_1.exe" )
ELSE()
    SET( OUTPUT_ROOT_DIR_0 "${WEBOTS_DIR}/projects/contests/robotstadium/controllers/nao_team_0/nao_team_0" )
    SET( OUTPUT_ROOT_DIR_1 "${WEBOTS_DIR}/projects/contests/robotstadium/controllers/nao_team_1/nao_team_1" )
ENDIF()

######### ADD_DEFINITIONS used by this target
ADD_DEFINITIONS(${WEBOTS_DEFINITIONS})

######### INCLUDE_DIRECTORIES required by this target
INCLUDE_DIRECTORIES( ${WEBOTS_INCLUDE_DIR}
)

######### Append required libraries to NUBOT_LINK_LIBRARIES
LIST(APPEND NUBOT_LINK_LIBRARIES ${WEBOTS_LIBRARIES})

IF (${CMAKE_SYSTEM_NAME} STREQUAL Darwin)
    SET(CMAKE_EXE_LINKER_FLAGS "-arch i386" CACHE STRING "os-x arch" FORCE)
    SET(CMAKE_CXX_FLAGS "-arch i386" CACHE STRING "os-x arch" FORCE)
ENDIF (${CMAKE_SYSTEM_NAME} STREQUAL Darwin)

######### Debug information
IF (DEBUG)
	MESSAGE(STATUS "Webots Package: ")
    MESSAGE(STATUS "-------directory: ${WEBOTS_DIR}")
    MESSAGE(STATUS "-------include:   ")
    FOREACH(loop_var ${WEBOTS_INCLUDE_DIR})
        MESSAGE(STATUS "-------------- ${loop_var}")
    ENDFOREACH(loop_var ${WEBOTS_INCLUDE_DIR})    
    MESSAGE(STATUS "       lib:       ")
    FOREACH(loop_var ${WEBOTS_LIBRARIES})
        MESSAGE(STATUS "-------------- ${loop_var}")
    ENDFOREACH(loop_var ${WEBOTS_LIBRARIES}) 
ENDIF (DEBUG)
