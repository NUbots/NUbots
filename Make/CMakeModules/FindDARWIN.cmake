# A package finder for DARWIN
#    Copyright (c) 2011 Jason Kulk
#    This file is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This file is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.

# This finds the location of the darwin.a and sets DARWIN_DIR, DARWIN_INCLUDE_DIR, DARWIN_LIBRARIES

IF (EXISTS /usr/lib/darwin.a)
    SET(DARWIN_DIR /usr)
ELSE()
    IF (EXISTS /usr/local/lib/darwin.a)
        SET(DARWIN_DIR /usr/local)
    ELSE()
        IF (EXISTS /darwin/Linux/lib/darwin.a)
            SET(DARWIN_DIR /darwin/Linux)
        ENDIF()
    ENDIF()
ENDIF()

IF (DARWIN_DIR)
    SET(DARWIN_INCLUDE_DIR ${DARWIN_DIR}/include CACHE FILEPATH "Cleared." FORCE)
    SET(DARWIN_LIBRARIES ${DARWIN_DIR}/lib/darwin.a CACHE FILEPATH "Cleared." FORCE)
ELSE()
    MESSAGE(ERROR darwin not found)
ENDIF()

MARK_AS_ADVANCED(
    DARWIN_INCLUDE_DIR
    DARWIN_LIBRARIES
)


