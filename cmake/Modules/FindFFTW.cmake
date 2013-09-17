# - Try to find FFTW
# Once done this will define
#
#  FFTW_FOUND - system has FFTW
#  FFTW_INCLUDE_DIRS - the FFTW include directory
#  FFTW_LIBRARIES - Link these to use FFTW
#
#  Copyright (c) 2013 Trent Houliston <trent@houliston.me>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

if (FFTW_LIBRARIES AND FFTW_INCLUDE_DIRS)
  # in cache already
  set(FFTW_FOUND TRUE)
else (FFTW_LIBRARIES AND FFTW_INCLUDE_DIRS)

  find_path(FFTW_INCLUDE_DIR
    NAMES
      fftw3.h
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
  )

  find_library(FFTW_LIBRARY
    NAMES
      fftw3
    PATHS
      /usr/lib
      /usr/lib/i386-linux-gnu
      /usr/local/lib
      /opt/local/lib
      /sw/lib
  )

  set(FFTW_INCLUDE_DIRS
    ${FFTW_INCLUDE_DIR}
  )

  if (FFTW_LIBRARY)
    set(FFTW_LIBRARIES
        ${FFTW_LIBRARIES}
        ${FFTW_LIBRARY}
    )
  endif (FFTW_LIBRARY)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(FFTW DEFAULT_MSG FFTW_LIBRARIES FFTW_INCLUDE_DIRS)

  # show the FFTW_INCLUDE_DIRS and FFTW_LIBRARIES variables only in the advanced view
  mark_as_advanced(FFTW_INCLUDE_DIRS FFTW_LIBRARIES)

endif (FFTW_LIBRARIES AND FFTW_INCLUDE_DIRS)

