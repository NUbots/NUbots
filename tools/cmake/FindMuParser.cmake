# - Try to find libMuParser
# Once done this will define
#
#  MUPARSER_FOUND - system has libMuParser
#  MUPARSER_INCLUDE_DIRS - the libMuParser include directory
#  MUPARSER_LIBRARIES - Link these to use libMuParser
#
#  Copyright (c) 2013 Trent Houliston <trent@houliston.me>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

if (MUPARSER_LIBRARIES AND MUPARSER_INCLUDE_DIRS)
  # in cache already
  set(MUPARSER_FOUND TRUE)
else (MUPARSER_LIBRARIES AND MUPARSER_INCLUDE_DIRS)

  find_path(MUPARSER_INCLUDE_DIR
    NAMES
      muParser.h
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
  )

  find_library(MUPARSER_LIBRARY
    NAMES
      muparser
    PATHS
      /usr/lib
      /usr/local/lib
      /opt/local/lib
      /sw/lib
  )

  set(MUPARSER_INCLUDE_DIRS
    ${MUPARSER_INCLUDE_DIR}
  )

  if (MUPARSER_LIBRARY)
    set(MUPARSER_LIBRARIES
        ${MUPARSER_LIBRARIES}
        ${MUPARSER_LIBRARY}
    )
  endif (MUPARSER_LIBRARY)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(MUPARSER DEFAULT_MSG MUPARSER_LIBRARIES MUPARSER_INCLUDE_DIRS)

  # show the MUPARSER_INCLUDE_DIRS and MUPARSER_LIBRARIES variables only in the advanced view
  mark_as_advanced(MUPARSER_INCLUDE_DIRS MUPARSER_LIBRARIES)

endif (MUPARSER_LIBRARIES AND MUPARSER_INCLUDE_DIRS)

