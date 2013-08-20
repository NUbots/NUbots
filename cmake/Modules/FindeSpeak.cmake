# - Try to find libeSpeak
# Once done this will define
#
#  ESPEAK_FOUND - system has libeSpeak
#  ESPEAK_INCLUDE_DIRS - the libeSpeak include directory
#  ESPEAK_LIBRARIES - Link these to use libeSpeak
#
#  Copyright (c) 2013 Trent Houliston <trent@houliston.me>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

if (ESPEAK_LIBRARIES AND ESPEAK_INCLUDE_DIRS)
  # in cache already
  set(ESPEAK_FOUND TRUE)
else (ESPEAK_LIBRARIES AND ESPEAK_INCLUDE_DIRS)

  find_path(ESPEAK_INCLUDE_DIR
    NAMES
      espeak/speak_lib.h
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
  )

  find_library(ESPEAK_LIBRARY
    NAMES
      espeak
    PATHS
      /usr/lib
      /usr/local/lib
      /opt/local/lib
      /sw/lib
  )

  set(ESPEAK_INCLUDE_DIRS
    ${ESPEAK_INCLUDE_DIR}
  )

  if (ESPEAK_LIBRARY)
    set(ESPEAK_LIBRARIES
        ${ESPEAK_LIBRARIES}
        ${ESPEAK_LIBRARY}
    )
  endif (ESPEAK_LIBRARY)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(ESPEAK DEFAULT_MSG ESPEAK_LIBRARIES ESPEAK_INCLUDE_DIRS)

  # show the ESPEAK_INCLUDE_DIRS and ESPEAK_LIBRARIES variables only in the advanced view
  mark_as_advanced(ESPEAK_INCLUDE_DIRS ESPEAK_LIBRARIES)

endif (ESPEAK_LIBRARIES AND ESPEAK_INCLUDE_DIRS)

