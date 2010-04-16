# A CMake file to configure the walk engine
#
#    Copyright (c) 2009 Jason Kulk
#    This file is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This file is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.

IF(DEBUG)
    MESSAGE(STATUS ${CMAKE_CURRENT_LIST_FILE})
ENDIF()

# I need to prefix each file and directory with the correct path
STRING(REPLACE "/cmake/ioconfig.cmake" "" THIS_SRC_DIR ${CMAKE_CURRENT_LIST_FILE})

############################ io options
SET( NUBOT_USE_NETWORK_GAMECONTROLLER
     ON
     CACHE BOOL
     "Set to ON to use network gamecontroller, set to OFF to use only the buttons")
SET( NUBOT_USE_NETWORK_TEAMINFO
     ON
     CACHE BOOL
     "Set to ON to use team packets, set to OFF to go solo")
SET( NUBOT_USE_NETWORK_JOBS
     ON
     CACHE BOOL
     "Set to ON to allow remote jobs to be executed, set to OFF to be ignorant of others")
SET( NUBOT_USE_NETWORK_DEBUGSTREAM
     ON
     CACHE BOOL
     "Set to ON to enable streaming of data of the network, set to OFF to keep quiet")
     
MARK_AS_ADVANCED(
    NUBOT_USE_NETWORK_GAMECONTROLLER
    NUBOT_USE_NETWORK_TEAMINFO
    NUBOT_USE_NETWORK_JOBS
    NUBOT_USE_NETWORK_DEBUGSTREAM
)

############################ ioconfig.h generation
CONFIGURE_FILE(
	"${THIS_SRC_DIR}/cmake/ioconfig.in"
  	"${THIS_SRC_DIR}/../../Autoconfig/ioconfig.h"
    ESCAPE_QUOTES
)