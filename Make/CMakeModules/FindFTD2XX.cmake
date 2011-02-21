# A package finder for FTD2XX
#    Copyright (c) 2010 Jason Kulk
#    This file is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This file is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.

# This finds the location of the libftd2xx and sets FTD2XX_DIR, FTD2XX_INCLUDE_DIR, FTD2XX_LIBRARIES, FTD2XX_DEFINITIONS 

IF (EXISTS /usr/lib/libftd2xx.so)
    SET(FTD2XX_DIR /usr)
ELSE()
    IF (EXISTS /usr/local/lib/libftd2xx.so)
        SET(FTD2XX_DIR /usr/local)
    ENDIF()
ENDIF()

IF (FTD2XX_DIR)
    SET(FTD2XX_INCLUDE_DIR ${FTD2XX_DIR}/include CACHE FILEPATH "Cleared." FORCE)
    SET(FTD2XX_LIBRARIES ${FTD2XX_DIR}/lib/libftd2xx.so CACHE FILEPATH "Cleared." FORCE)
    SET(FTD2XX_DEFINITIONS -DFOUND_FTD2XX=TRUE)
ELSE()
    MESSAGE(ERROR libftd2xx not found)
ENDIF()

MARK_AS_ADVANCED(
    FTD2XX_INCLUDE_DIR
    FTD2XX_LIBRARIES
)


