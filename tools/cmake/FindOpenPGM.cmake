# - Try to find OPENPGM
# Once done this will define
#
#  OPENPGM_FOUND - system has OPENPGM
#  OPENPGM_INCLUDE_DIRS - the OPENPGM include directory
#  OPENPGM_LIBRARIES - Link these to use OPENPGM
#  OPENPGM_DEFINITIONS - Compiler switches required for using OPENPGM
#
#  Copyright (c) 2011 Lee Hambley <lee.hambley@gmail.com>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

if (OPENPGM_LIBRARIES)
  # in cache already
  set(OPENPGM_FOUND TRUE)
else (OPENPGM_LIBRARIES)

  find_library(OPENPGM_LIBRARY
    NAMES
      pgm
    PATHS
      /usr/lib
      /usr/local/lib
      /opt/local/lib
      /sw/lib
  )

  if (OPENPGM_LIBRARY)
    set(OPENPGM_LIBRARIES
        ${OPENPGM_LIBRARIES}
        ${OPENPGM_LIBRARY}
    )
  endif (OPENPGM_LIBRARY)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(OPENPGM DEFAULT_MSG OPENPGM_LIBRARIES)

  # show the OPENPGM_INCLUDE_DIRS and OPENPGM_LIBRARIES variables only in the advanced view
  mark_as_advanced(OPENPGM_LIBRARIES)

endif (OPENPGM_LIBRARIES)

