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
OPTION( NUBOT_USE_NETWORK_GAMECONTROLLER
        "Set to ON to use network gamecontroller, set to OFF to use only the buttons"
        ON)
OPTION( NUBOT_USE_NETWORK_TEAMINFO
        "Set to ON to use team packets, set to OFF to go solo"
        ON)
OPTION( NUBOT_USE_NETWORK_JOBS
        "Set to ON to allow remote jobs to be executed, set to OFF to be ignorant of others"
        ON)
OPTION( NUBOT_USE_NETWORK_DEBUGSTREAM
        "Set to ON to enable streaming of data of the network, set to OFF to keep quiet"
        ON)

############################ ioconfig.h generation
CONFIGURE_FILE(
	"${THIS_SRC_DIR}/cmake/ioconfig.in"
  	"${THIS_SRC_DIR}/../../Autoconfig/ioconfig.h"
    ESCAPE_QUOTES
)