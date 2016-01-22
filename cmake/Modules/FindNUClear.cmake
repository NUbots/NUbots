# - Try to find NUClear
# Once done this will define
#
#  NUCLEAR_FOUND - system has NUClear
#  NUCLEAR_INCLUDE_DIRS - the NUClear include directory
#  NUCLEAR_LIBRARIES - Link these to use NUClear
#
#  Copyright (c) 2013 Trent Houliston <trent@houliston.me>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

if (NUCLEAR_LIBRARIES AND NUCLEAR_INCLUDE_DIRS)
  # in cache already
  set(NUCLEAR_FOUND TRUE)
else (NUCLEAR_LIBRARIES AND NUCLEAR_INCLUDE_DIRS)

  find_path(NUCLEAR_INCLUDE_DIR
    NAMES
      nuclear
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
      $ENV{NUCLEAR_LOCATION}/include
      $ENV{SHARED_LIBRARIES}/${TARGET_ARCHITECTURE}/include
  )

  find_library(NUCLEAR_LIBRARY
    NAMES
      nuclear
    PATHS
      /usr/lib
      /usr/local/lib
      /opt/local/lib
      /sw/lib
      $ENV{SHARED_LIBRARIES}/${TARGET_ARCHITECTURE}/lib
  )

  set(NUCLEAR_INCLUDE_DIRS
    ${NUCLEAR_INCLUDE_DIR}
  )

  if (NUCLEAR_LIBRARY)
    set(NUCLEAR_LIBRARIES
        ${NUCLEAR_LIBRARIES}
        ${NUCLEAR_LIBRARY}
    )
  endif (NUCLEAR_LIBRARY)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(NUClear DEFAULT_MSG NUCLEAR_LIBRARIES NUCLEAR_INCLUDE_DIRS)

  # show the NUCLEAR_INCLUDE_DIRS and NUCLEAR_LIBRARIES variables only in the advanced view
  mark_as_advanced(NUCLEAR_INCLUDE_DIRS NUCLEAR_LIBRARIES)

endif (NUCLEAR_LIBRARIES AND NUCLEAR_INCLUDE_DIRS)

