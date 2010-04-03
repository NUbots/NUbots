# A package finder for WEBOTS
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


# Find the location of Webots
IF ("x$ENV{WEBOTS_HOME}x" STREQUAL "xx")
    IF (${CMAKE_SYSTEM_NAME} STREQUAL Windows)
        # I am going to assume that Webots is in the default location
        IF (EXISTS "/Program Files/Webots")
            SET (ENV{WEBOTS_HOME} "/Program Files/Webots")
        ENDIF (EXISTS "/Program Files/Webots")
        IF (EXISTS "/Program Files (x86)/Webots")
            SET (ENV{WEBOTS_HOME} "/Program Files (x86)/Webots")
        ENDIF (EXISTS "/Program Files (x86)/Webots")
        
    ELSE (${CMAKE_SYSTEM_NAME} STREQUAL Windows)
        IF (${CMAKE_SYSTEM_NAME} STREQUAL Darwin)
            MESSAGE(STATUS "Looking for Webots on Darwin")
            # There are only two logical places to put Webots on Darwin
            # /Application/webots or ~/webots
            IF (EXISTS /Applications/webots)
                SET (ENV{WEBOTS_HOME} /Applications/webots)
            ENDIF (EXISTS /Applications/webots)

            IF (EXISTS $ENV{HOME}/webots)
                SET (ENV{WEBOTS_HOME} $ENV{HOME}/webots)
            ENDIF (EXISTS $ENV{HOME}/webots)

        ELSE (${CMAKE_SYSTEM_NAME} STREQUAL Darwin)
            IF (${CMAKE_SYSTEM_NAME} STREQUAL Linux)
                # There are only two logical places to put Webots on Linux 
                # /usr/local/webots or ~/webots
                
                IF (EXISTS /usr/local/webots)
                    SET (ENV{WEBOTS_HOME} /usr/local/webots)
                ENDIF (EXISTS /usr/local/webots)

                IF (EXISTS $ENV{HOME}/webots)
                    SET (ENV{WEBOTS_HOME} $ENV{HOME}/webots)
                ENDIF (EXISTS $ENV{HOME}/webots)

            ELSE (${CMAKE_SYSTEM_NAME} STREQUAL Linux)
                MESSAGE(ERROR "I didn't know Webots could run on ${CMAKE_SYSTEM_NAME}")
            ENDIF (${CMAKE_SYSTEM_NAME} STREQUAL Linux)
        ENDIF (${CMAKE_SYSTEM_NAME} STREQUAL Darwin)
    ENDIF (${CMAKE_SYSTEM_NAME} STREQUAL Windows)
ENDIF ("x$ENV{WEBOTS_HOME}x" STREQUAL "xx")

## See if we were successful in finding webots
IF ("x$ENV{WEBOTS_HOME}x" STREQUAL "xx")
    MESSAGE(ERROR "Unable to find Webots. You will need to set ENV{WEBOTS_HOME}")
ELSE ("x$ENV{WEBOTS_HOME}x" STREQUAL "xx")
	SET(WEBOTS_FOUND TRUE)
ENDIF ("x$ENV{WEBOTS_HOME}x" STREQUAL "xx")

# set the directory and then includes
SET(WEBOTS_DIR $ENV{WEBOTS_HOME})
SET(WEBOTS_INCLUDE_DIR  "${WEBOTS_DIR}/include/controller/cpp"
                        "${WEBOTS_DIR}/include"
                        "${WEBOTS_DIR}/projects/contests/nao_robocup/controllers"
                        CACHE FILEPATH "Cleared." FORCE)

# now set the platform dependant library
IF (${CMAKE_SYSTEM_NAME} STREQUAL Windows)
    SET(WEBOTS_LIBRARIES "${WEBOTS_DIR}/lib/libCppController.a" CACHE FILEPATH "Cleared." FORCE)

ELSE (${CMAKE_SYSTEM_NAME} STREQUAL Windows)
    IF (${CMAKE_SYSTEM_NAME} STREQUAL Darwin)
        SET(WEBOTS_LIBRARIES "${WEBOTS_DIR}/lib/libCppController.dylib" CACHE FILEPATH "Cleared." FORCE)

    ELSE (${CMAKE_SYSTEM_NAME} STREQUAL Darwin)
        SET(WEBOTS_LIBRARIES "${WEBOTS_DIR}/lib/libCppController.so" CACHE FILEPATH "Cleared." FORCE)

    ENDIF (${CMAKE_SYSTEM_NAME} STREQUAL Darwin)
ENDIF (${CMAKE_SYSTEM_NAME} STREQUAL Windows)

# finally set the webots definition
SET(WEBOTS_DEFINITIONS -DFOUND_WEBOTS=TRUE)

MARK_AS_ADVANCED(
    WEBOTS_INCLUDE_DIR
    WEBOTS_LIBRARIES
)


