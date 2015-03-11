# - Try to find libOpenBLAS
# Once done this will define
#
#  OPENBLAS_FOUND - system has libOpenBLAS
#  OPENBLAS_INCLUDE_DIRS - the libOpenBLAS include directory
#  OPENBLAS_LIBRARIES - Link these to use libOpenBLAS
#
#  Copyright (c) 2013 Trent Houliston <trent@houliston.me>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

if (OPENBLAS_LIBRARIES AND OPENBLAS_INCLUDE_DIRS)
  # in cache already
  set(OPENBLAS_FOUND TRUE)
else (OPENBLAS_LIBRARIES AND OPENBLAS_INCLUDE_DIRS)

  find_path(OPENBLAS_INCLUDE_DIR
    NAMES
      cblas.h
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
  )

  find_library(OPENBLAS_LIBRARY
    NAMES
      openblas
    PATHS
      /usr/lib
      /usr/local/lib
      /opt/local/lib
      /sw/lib
  )

  set(OPENBLAS_INCLUDE_DIRS
    ${OPENBLAS_INCLUDE_DIR}
  )

  if (OPENBLAS_LIBRARY)
    set(OPENBLAS_LIBRARIES
        ${OPENBLAS_LIBRARIES}
        ${OPENBLAS_LIBRARY}
    )
  endif (OPENBLAS_LIBRARY)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(OPENBLAS DEFAULT_MSG OPENBLAS_LIBRARIES OPENBLAS_INCLUDE_DIRS)

  # show the OPENBLAS_INCLUDE_DIRS and OPENBLAS_LIBRARIES variables only in the advanced view
  mark_as_advanced(OPENBLAS_INCLUDE_DIRS OPENBLAS_LIBRARIES)

endif (OPENBLAS_LIBRARIES AND OPENBLAS_INCLUDE_DIRS)
