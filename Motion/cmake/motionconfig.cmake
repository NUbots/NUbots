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
SET( NUBOT_USE_MOTION_HEAD
	ON
	CACHE BOOL
    "Set to ON to use head, set to OFF to not move the head")
SET( NUBOT_USE_MOTION_WALK
	ON
	CACHE BOOL
    "Set to ON to use walk, set to OFF to have no walk")
SET( NUBOT_USE_MOTION_KICK
	OFF 
	CACHE BOOL
    "Set to ON to use kick, set to OFF to not have kick")
SET( NUBOT_USE_MOTION_BLOCK
	OFF 
	CACHE BOOL
    "Set to ON to use block, set to OFF to not have any blocks")
SET( NUBOT_USE_MOTION_SAVE
	OFF 
	CACHE BOOL
    "Set to ON to use save, set to OFF to not have any saves")
SET( NUBOT_USE_MOTION_SCRIPT
	OFF 
	CACHE BOOL
    "Set to ON to use script, set to OFF to not have any scripts")
SET( NUBOT_USE_MOTION_GETUP
	OFF
	CACHE BOOL
	"Set to ON to use getups, set to OFF to stay on the ground")
SET( NUBOT_USE_MOTION_FALL_PROTECTION
	OFF
	CACHE BOOL
	"Set to ON to use fall protection, set to OFF to just turn off stiffness when falling")

MARK_AS_ADVANCED(
	NUBOT_USE_MOTION_HEAD
	NUBOT_USE_MOTION_WALK
	NUBOT_USE_MOTION_KICK
    NUBOT_USE_MOTION_BLOCK
    NUBOT_USE_MOTION_SAVE
    NUBOT_USE_MOTION_SCRIPT
	NUBOT_USE_MOTION_GETUP
	NUBOT_USE_MOTION_FALL_PROTECTION
)

	

############################ motionconfig.h generation
CONFIGURE_FILE(
	"${THIS_SRC_DIR}/cmake/motionconfig.in"
  	"${THIS_SRC_DIR}/../Autoconfig/motionconfig.h"
    ESCAPE_QUOTES
)