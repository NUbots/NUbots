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
STRING(REPLACE "/cmake/motionconfig.cmake" "" THIS_SRC_DIR ${CMAKE_CURRENT_LIST_FILE})

############################ walk engine options
OPTION(
        USE_MOTION_HEAD
        "Set to ON to use head, set to OFF to not move the head"
        ON
)
OPTION(
        USE_MOTION_WALK
        "Set to ON to use walk, set to OFF to have no walk"
        ON
)
OPTION(
        USE_MOTION_KICK
        "Set to ON to use kick, set to OFF to not have kick"
        OFF
)

############################ motionconfig.h generation
CONFIGURE_FILE(
	"${THIS_SRC_DIR}/cmake/motionconfig.in"
  	"${THIS_SRC_DIR}/../Autoconfig/motionconfig.h"
    ESCAPE_QUOTES
)