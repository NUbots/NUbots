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

############################ walk engine options
OPTION(
        USE_WALK
        "Set to ON to use walk, set to OFF to have no walk"
        ON
)

OPTION(
	USE_WALKOPTIMISER
	"Set to ON to use the walk optimiser, set to OFF for game"
	OFF
)

OPTION(
        USE_WALK_JWALK
        "Set to ON to use jwalk, set to OFF use something else"
        OFF
)

OPTION(
        USE_WALK_JUPPWALK
        "Set to ON to use juppwalk, set to OFF use something else"
        ON
)

OPTION(
        USE_WALK_NBWALK
        "Set to ON to use nbwalk, set to OFF use something else"
        OFF
)

OPTION(
        USE_WALK_VSCWALK
        "Set to ON to use vscwalk, set to OFF use something else"
        OFF
)

############################ walkconfig.h generation
CONFIGURE_FILE(
	"${THIS_SRC_DIR}/cmake/walkconfig.in"
  	"${THIS_SRC_DIR}/../../Autoconfig/walkconfig.h"
    ESCAPE_QUOTES
)