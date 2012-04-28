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
STRING(REPLACE "/cmake/walkconfig.cmake" "" THIS_SRC_DIR ${CMAKE_CURRENT_LIST_FILE})

############################# default walk engines for specific platforms
####### NAO
IF (${TARGET_ROBOT} STREQUAL NAO)
    SET( NUBOT_USE_MOTION_WALK_ALWALK
         ON
         CACHE BOOL
         "Set to ON to use almotion's walk, set to OFF use something else")
ELSE()
    SET( NUBOT_USE_MOTION_WALK_ALWALK
         OFF
         CACHE BOOL
         "Set to ON to use almotion's walk, set to OFF use something else")
ENDIF()

####### NAOWebots
IF (${TARGET_ROBOT} STREQUAL NAOWEBOTS)
    SET( NUBOT_USE_MOTION_WALK_NBWALK
         ON
         CACHE BOOL
         "Set to ON to use nbwalk, set to OFF use something else")
ELSE()
    SET( NUBOT_USE_MOTION_WALK_NBWALK
         OFF
         CACHE BOOL
         "Set to ON to use nbwalk, set to OFF use something else")
ENDIF()

####### Cycloid
IF (${TARGET_ROBOT} STREQUAL CYCLOID)
    SET( NUBOT_USE_MOTION_WALK_JUPPWALK
         ON
         CACHE BOOL
         "Set to ON to use juppwalk, set to OFF use something else")
ELSE()
    SET( NUBOT_USE_MOTION_WALK_JUPPWALK
         OFF
         CACHE BOOL
         "Set to ON to use juppwalk, set to OFF use something else")
ENDIF()

####### Cycloid
IF (${TARGET_ROBOT} STREQUAL BEAR)
    SET( NUBOT_USE_MOTION_WALK_BEARWALK
         ON
         CACHE BOOL
         "Set to ON to use bearwalk, set to OFF use something else")
ELSE()
    SET( NUBOT_USE_MOTION_WALK_BEARWALK
         OFF
         CACHE BOOL
         "Set to ON to use bearwalk, set to OFF use something else")
ENDIF()

####### Darwin
IF (${TARGET_ROBOT} STREQUAL DARWIN)
    SET( NUBOT_USE_MOTION_WALK_DARWINWALK
         ON
         CACHE BOOL
         "Set to ON to use bearwalk, set to OFF use something else")
ELSE()
    SET( NUBOT_USE_MOTION_WALK_DARWINWALK
         OFF
         CACHE BOOL
         "Set to ON to use bearwalk, set to OFF use something else")
ENDIF()

############################ other walk engines

SET( NUBOT_USE_MOTION_WALK_VSCWALK
     OFF
     CACHE BOOL
     "Set to ON to use vscwalk, set to OFF use something else")
     
SET( NUBOT_USE_MOTION_WALK_JWALK
     OFF 
     CACHE BOOL
     "Set to ON to use jwalk, set to OFF use something else")

SET( NUBOT_USE_MOTION_WALK_BWALK
     OFF 
     CACHE BOOL
     "Set to ON to use bwalk, set to OFF use something else")

MARK_AS_ADVANCED(
	NUBOT_USE_MOTION_WALK_JWALK
	NUBOT_USE_MOTION_WALK_BWALK
	NUBOT_USE_MOTION_WALK_JUPPWALK
	NUBOT_USE_MOTION_WALK_ALWALK
	NUBOT_USE_MOTION_WALK_NBWALK
	NUBOT_USE_MOTION_WALK_VSCWALK
	NUBOT_USE_MOTION_WALK_ALWALK
    	NUBOT_USE_MOTION_WALK_BEARWALK
	NUBOT_USE_MOTION_WALK_DARWINWALK
)
	

############################ walkconfig.h generation
CONFIGURE_FILE(
	"${THIS_SRC_DIR}/cmake/walkconfig.in"
  	"${THIS_SRC_DIR}/../../Autoconfig/walkconfig.h"
    ESCAPE_QUOTES
)
