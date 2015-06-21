# - Try to find libPortAudio
# Once done this will define
#
#  PORTAUDIO_FOUND - system has libPortAudio
#  PORTAUDIO_INCLUDE_DIRS - the libPortAudio include directory
#  PORTAUDIO_LIBRARIES - Link these to use libPortAudio
#
#  Copyright (c) 2013 Trent Houliston <trent@houliston.me>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

if (PORTAUDIO_LIBRARIES AND PORTAUDIO_INCLUDE_DIRS)
  # in cache already
  set(PORTAUDIO_FOUND TRUE)
else (PORTAUDIO_LIBRARIES AND PORTAUDIO_INCLUDE_DIRS)

  find_path(PORTAUDIO_INCLUDE_DIR
    NAMES
      portaudio.h
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
  )

  find_library(PORTAUDIO_LIBRARY
    NAMES
      portaudio
    PATHS
      /usr/lib
      /usr/local/lib
      /opt/local/lib
      /sw/lib
  )

  set(PORTAUDIO_INCLUDE_DIRS
    ${PORTAUDIO_INCLUDE_DIR}
  )

  if (PORTAUDIO_LIBRARY)
    set(PORTAUDIO_LIBRARIES
        ${PORTAUDIO_LIBRARIES}
        ${PORTAUDIO_LIBRARY}
    )
  endif (PORTAUDIO_LIBRARY)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(PORTAUDIO DEFAULT_MSG PORTAUDIO_LIBRARIES PORTAUDIO_INCLUDE_DIRS)

  # show the PORTAUDIO_INCLUDE_DIRS and PORTAUDIO_LIBRARIES variables only in the advanced view
  mark_as_advanced(PORTAUDIO_INCLUDE_DIRS PORTAUDIO_LIBRARIES)

endif (PORTAUDIO_LIBRARIES AND PORTAUDIO_INCLUDE_DIRS)

