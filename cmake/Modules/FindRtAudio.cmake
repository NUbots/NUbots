# - Try to find RtAudio
# Once done this will define
#
#  RTAUDIO_FOUND - system has RtAudio
#  RTAUDIO_INCLUDE_DIRS - the RtAudio include directory
#  RTAUDIO_LIBRARIES - Link these to use RtAudio
#
#  Copyright (c) 2013 Trent Houliston <trent@houliston.me>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

if (RTAUDIO_LIBRARIES AND RTAUDIO_INCLUDE_DIRS)
  # in cache already
  set(RTAUDIO_FOUND TRUE)
else (RTAUDIO_LIBRARIES AND RTAUDIO_INCLUDE_DIRS)

  find_path(RTAUDIO_INCLUDE_DIR
    NAMES
      RtAudio.h
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
  )

  find_library(RTAUDIO_LIBRARY
    NAMES
      rtaudio
    PATHS
      /usr/lib
      /usr/lib/i386-linux-gnu
      /usr/local/lib
      /opt/local/lib
      /sw/lib
  )

  set(RTAUDIO_INCLUDE_DIRS
    ${RTAUDIO_INCLUDE_DIR}
  )

  if (RTAUDIO_LIBRARY)
    set(RTAUDIO_LIBRARIES
        ${RTAUDIO_LIBRARIES}
        ${RTAUDIO_LIBRARY}
    )
  endif (RTAUDIO_LIBRARY)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(RtAudio DEFAULT_MSG RTAUDIO_LIBRARIES RTAUDIO_INCLUDE_DIRS)

  # show the RTAUDIO_INCLUDE_DIRS and RTAUDIO_LIBRARIES variables only in the advanced view
  mark_as_advanced(RTAUDIO_INCLUDE_DIRS RTAUDIO_LIBRARIES)

endif (RTAUDIO_LIBRARIES AND RTAUDIO_INCLUDE_DIRS)

