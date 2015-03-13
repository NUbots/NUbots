# - Try to find Quex
# Once done this will define
#
#  QUEX_FOUND - system has QUEX
#  QUEX_INCLUDE_DIRS - the QUEX include directory
#  QUEX_DEFINITIONS - Compiler switches required for using QUEX
#
#  Copyright (c) 2011 Lee Hambley <lee.hambley@gmail.com>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

if (QUEX_INCLUDE_DIRS)
  # in cache already
  set(QUEX_FOUND TRUE)
else (QUEX_INCLUDE_DIRS)

  find_path(QUEX_INCLUDE_DIR
    NAMES
      quex/code_base/analyzer/C-adaptions.h
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
  )

  set(QUEX_INCLUDE_DIRS
    ${QUEX_INCLUDE_DIR}
  )
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(QUEX DEFAULT_MSG QUEX_INCLUDE_DIRS)

  # show the QUEX_INCLUDE_DIRS variables only in the advanced view
  mark_as_advanced(QUEX_INCLUDE_DIRS)

endif (QUEX_INCLUDE_DIRS)
