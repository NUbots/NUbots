# - Try to find libYAMLcpp
# Once done this will define
#
#  YAML_FOUND - system has libYAMLcpp
#  YAML_INCLUDE_DIRS - the libYAMLcpp include directory
#  YAML_LIBRARIES - Link these to use libYAMLcpp
#
#  Copyright (c) 2013 Trent Houliston <trent@houliston.me>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

if (YAML_LIBRARIES AND YAML_INCLUDE_DIRS)
  # in cache already
  set(YAML_FOUND TRUE)
else (YAML_LIBRARIES AND YAML_INCLUDE_DIRS)

  find_path(YAML_INCLUDE_DIR
    NAMES
      yaml-cpp
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
  )

  find_library(YAML_LIBRARY
    NAMES
      yaml-cpp
    PATHS
      /usr/lib
      /usr/local/lib
      /opt/local/lib
      /sw/lib
  )

  set(YAML_INCLUDE_DIRS
    ${YAML_INCLUDE_DIR}
  )

  if (YAML_LIBRARY)
    set(YAML_LIBRARIES
        ${YAML_LIBRARIES}
        ${YAML_LIBRARY}
    )
  endif (YAML_LIBRARY)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(YAML DEFAULT_MSG YAML_LIBRARIES YAML_INCLUDE_DIRS)

  # show the YAML_INCLUDE_DIRS and YAML_LIBRARIES variables only in the advanced view
  mark_as_advanced(YAML_INCLUDE_DIRS YAML_LIBRARIES)

endif (YAML_LIBRARIES AND YAML_INCLUDE_DIRS)

