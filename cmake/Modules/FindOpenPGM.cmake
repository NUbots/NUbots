# - Try to find OPENPGM
# Once done this will define
#
#  OPENPGM_FOUND - system has OPENPGM
#  OPENPGM_INCLUDE_DIRS - the OPENPGM include directory
#  OPENPGM_LIBRARIES - Link these to use OPENPGM
#  OPENPGM_DEFINITIONS - Compiler switches required for using OPENPGM
#
#  Copyright (c) 2013 Trent Houliston <trent@houliston.me>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#


if (OPENPGM_LIBRARIES AND OPENPGM_INCLUDE_DIRS)
  # in cache already
  set(OPENPGM_FOUND TRUE)
else (OPENPGM_LIBRARIES AND OPENPGM_INCLUDE_DIRS)
  find_path(OPENPGM_INCLUDE_DIR
    NAMES
      pgm/pgm.h
    PATHS
    ${OPENPGM_DIR}/include
    $ENV{OPENPGM_DIR}/include
    $ENV{OPENPGM_DIR}
    ${DELTA3D_EXT_DIR}/inc
    $ENV{DELTA_ROOT}/ext/inc
    $ENV{DELTA_ROOT}
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/include
    /usr/include
    /usr/include/gdal
    /sw/include # Fink
    /opt/local/include # DarwinPorts
    /opt/csw/include # Blastwave
    /opt/include
    [HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Session\ Manager\\Environment;OSG_ROOT]/include
    /usr/freeware/include
      
  )

  find_library(OPENPGM_LIBRARY
    NAMES
      pgm
    PATHS
    ${OPENPGM_DIR}/lib
    $ENV{OPENPGM_DIR}/lib
    $ENV{OPENPGM_DIR}
    ${DELTA3D_EXT_DIR}/lib
    $ENV{DELTA_ROOT}/ext/lib
    $ENV{DELTA_ROOT}
    $ENV{OSG_ROOT}/lib
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/lib
    /usr/lib
    /sw/lib
    /opt/local/lib
    /opt/csw/lib
    /opt/lib
    /usr/freeware/lib64
  )

  find_library(OPENPGM_LIBRARY_DEBUG
    NAMES
      pgmd
    PATHS
    ${OPENPGM_DIR}/lib
    $ENV{OPENPGM_DIR}/lib
    $ENV{OPENPGM_DIR}
    ${DELTA3D_EXT_DIR}/lib
    $ENV{DELTA_ROOT}/ext/lib
    $ENV{DELTA_ROOT}
    $ENV{OSG_ROOT}/lib
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/lib
    /usr/lib
    /sw/lib
    /opt/local/lib
    /opt/csw/lib
    /opt/lib
    /usr/freeware/lib64
  )

  set(OPENPGM_INCLUDE_DIRS
    ${OPENPGM_INCLUDE_DIR}
  )
  set(OPENPGM_LIBRARIES
    ${OPENPGM_LIBRARY}
)

  if (OPENPGM_INCLUDE_DIRS AND OPENPGM_LIBRARIES)
     set(OPENPGM_FOUND TRUE)
  endif (OPENPGM_INCLUDE_DIRS AND OPENPGM_LIBRARIES)

  if (OPENPGM_FOUND)
    if (NOT OPENPGM_FIND_QUIETLY)
      message(STATUS "Found OPENPGM: ${OPENPGM_LIBRARIES}")
    endif (NOT OPENPGM_FIND_QUIETLY)
  else (OPENPGM_FOUND)
    if (OPENPGM_FIND_REQUIRED)
      message(FATAL_ERROR "Could not find OPENPGM")
    endif (OPENPGM_FIND_REQUIRED)
  endif (OPENPGM_FOUND)

  # show the OPENPGM_INCLUDE_DIRS and OPENPGM_LIBRARIES variables only in the advanced view
  mark_as_advanced(OPENPGM_INCLUDE_DIRS OPENPGM_LIBRARIES)

endif (OPENPGM_LIBRARIES AND OPENPGM_INCLUDE_DIRS)
