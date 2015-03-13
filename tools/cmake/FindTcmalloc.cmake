# - Try to find libTcmalloc
# Once done this will define
#
#  TCMALLOC_FOUND - system has libeSpeak
#  TCMALLOC_INCLUDE_DIRS - the libeSpeak include directory
#  TCMALLOC_LIBRARIES - Link these to use libeSpeak
#
#  Copyright (c) 2013 Trent Houliston <trent@houliston.me>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

if (TCMALLOC_LIBRARIES AND TCMALLOC_INCLUDE_DIRS)
  # in cache already
  set(TCMALLOC_FOUND TRUE)
else (TCMALLOC_LIBRARIES AND TCMALLOC_INCLUDE_DIRS)

  find_path(TCMALLOC_INCLUDE_DIR
    NAMES
      google/tcmalloc.h NO_DEFAULT_PATH PATHS
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
  )

  find_library(TCMALLOC_LIBRARY
    NAMES
      tcmalloc
      tcmalloc_minimal
    PATHS
      /usr/lib
      /usr/local/lib
      /opt/local/lib
      /sw/lib
  )

  set(TCMALLOC_INCLUDE_DIRS
    ${TCMALLOC_INCLUDE_DIR}
  )

  if (TCMALLOC_LIBRARY)
    set(TCMALLOC_LIBRARIES
        ${TCMALLOC_LIBRARIES}
        ${TCMALLOC_LIBRARY}
    )
  endif (TCMALLOC_LIBRARY)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(TCMALLOC DEFAULT_MSG TCMALLOC_LIBRARIES TCMALLOC_INCLUDE_DIRS)

  # show the TCMALLOC_INCLUDE_DIRS and TCMALLOC_LIBRARIES variables only in the advanced view
  mark_as_advanced(TCMALLOC_INCLUDE_DIRS TCMALLOC_LIBRARIES)

endif (TCMALLOC_LIBRARIES AND TCMALLOC_INCLUDE_DIRS)

