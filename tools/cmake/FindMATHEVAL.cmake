# - Try to find libmatheval
# Once done this will define
#
#  MATHEVAL_FOUND - system has libeSpeak
#  MATHEVAL_INCLUDE_DIRS - the libeSpeak include directory
#  MATHEVAL_LIBRARIES - Link these to use libeSpeak
#
#  Copyright (c) 2013 Trent Houliston <trent@houliston.me>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

if (MATHEVAL_LIBRARIES AND MATHEVAL_INCLUDE_DIRS)
  # in cache already
  set(MATHEVAL_FOUND TRUE)
else (MATHEVAL_LIBRARIES AND MATHEVAL_INCLUDE_DIRS)

  find_path(MATHEVAL_INCLUDE_DIR
    NAMES
      matheval.h
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
  )

  find_library(MATHEVAL_LIBRARY
    NAMES
      matheval
    PATHS
      /usr/lib
      /usr/local/lib
      /opt/local/lib
      /sw/lib
  )

  set(MATHEVAL_INCLUDE_DIRS
    ${MATHEVAL_INCLUDE_DIR}
  )

  if (MATHEVAL_LIBRARY)
    set(MATHEVAL_LIBRARIES
        ${MATHEVAL_LIBRARIES}
        ${MATHEVAL_LIBRARY}
    )
  endif (MATHEVAL_LIBRARY)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(MATHEVAL DEFAULT_MSG MATHEVAL_LIBRARIES MATHEVAL_INCLUDE_DIRS)

  # show the MATHEVAL_INCLUDE_DIRS and MATHEVAL_LIBRARIES variables only in the advanced view
  mark_as_advanced(MATHEVAL_INCLUDE_DIRS MATHEVAL_LIBRARIES)

endif (MATHEVAL_LIBRARIES AND MATHEVAL_INCLUDE_DIRS)
