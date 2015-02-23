# - Try to find the bibtool binary
# Once done this will define
#
#  BIBTOOL_FOUND - system has FFTW
#  BIBTOOL_EXEC - the bibtool executable
#
#  Copyright (c) 2013 Trent Houliston <trent@houliston.me>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

if (BIBTOOL_EXEC)
  # in cache already
  set(BIBTOOL_FOUND TRUE)
else (BIBTOOL_EXEC)

  find_program(BIBTOOL_EXEC
    NAMES
      bibtool
    PATHS
      /usr/bin
      /usr/local/bin
  )

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(BIBTOOL DEFAULT_MSG BIBTOOL_EXEC)

  # show the FFTW_INCLUDE_DIRS and FFTW_LIBRARIES variables only in the advanced view
  mark_as_advanced(BIBTOOL_EXEC)

endif (BIBTOOL_EXEC)

