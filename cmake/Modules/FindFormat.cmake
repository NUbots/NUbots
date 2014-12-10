if (FORMAT_LIBRARIES AND FORMAT_INCLUDE_DIRS)
  # in cache already
  set(FORMAT_FOUND TRUE)
else (FORMAT_INCLUDE_DIRS)

  find_path(FORMAT_INCLUDE_DIR
    NAMES
      format.h
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
  )

  find_library(FORMAT_LIBRARY
    NAMES
      format
    PATHS
      /usr/lib
      /usr/local/lib
      /opt/local/lib
      /sw/lib
  )

  set(FORMAT_INCLUDE_DIRS
    ${FORMAT_INCLUDE_DIR}
  )

  if (FORMAT_LIBRARY)
    set(FORMAT_LIBRARIES
        ${FORMAT_LIBRARIES}
        ${FORMAT_LIBRARY}
    )
  endif (FORMAT_LIBRARY)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(FORMAT DEFAULT_MSG FORMAT_LIBRARIES FORMAT_INCLUDE_DIRS)

  # show the CATCH_INCLUDE_DIRS variables only in the advanced view
  mark_as_advanced(FORMAT_INCLUDE_DIRS FORMAT_LIBRARIES)

endif (FORMAT_LIBRARIES AND FORMAT_INCLUDE_DIRS)

