# - Try to find SndFile
# Once done this will define
#
#  SNDFILE_FOUND - system has SndFile
#  SNDFILE_INCLUDE_DIRS - the SndFile include directory
#  SNDFILE_LIBRARIES - Link these to use SndFile
#
#  Copyright (c) 2013 Trent Houliston <trent@houliston.me>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

if (SNDFILE_LIBRARIES AND SNDFILE_INCLUDE_DIRS)
  # in cache already
  set(SNDFILE_FOUND TRUE)
else (SNDFILE_LIBRARIES AND SNDFILE_INCLUDE_DIRS)

  find_path(SNDFILE_INCLUDE_DIR
    NAMES
      sndfile.hh
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
  )

  find_library(SNDFILE_LIBRARY
    NAMES
      sndfile
    PATHS
      /usr/lib
      /usr/lib/i386-linux-gnu
      /usr/local/lib
      /opt/local/lib
      /sw/lib
  )

  set(SNDFILE_INCLUDE_DIRS
    ${SNDFILE_INCLUDE_DIR}
  )

  if (SNDFILE_LIBRARY)
    set(SNDFILE_LIBRARIES
        ${SNDFILE_LIBRARIES}
        ${SNDFILE_LIBRARY}
    )
  endif (SNDFILE_LIBRARY)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(SndFile DEFAULT_MSG SNDFILE_LIBRARIES SNDFILE_INCLUDE_DIRS)

  # show the SNDFILE_INCLUDE_DIRS and SNDFILE_LIBRARIES variables only in the advanced view
  mark_as_advanced(SNDFILE_INCLUDE_DIRS SNDFILE_LIBRARIES)

endif (SNDFILE_LIBRARIES AND SNDFILE_INCLUDE_DIRS)

