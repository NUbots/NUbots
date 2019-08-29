# * Try to find libGFortran Once done this will define
#
# LIBGFORTRAN_FOUND - system has libGFortran LIBGFORTRAN_LIBRARIES - Link these to use libGFortran
#
# Copyright (c) 2013 Trent Houliston <trent@houliston.me>
#
# Redistribution and use is allowed according to the terms of the New BSD license. For details see the accompanying
# COPYING-CMAKE-SCRIPTS file.
#

if(LIBGFORTRAN_LIBRARIES)
  # in cache already
  set(LIBGFORTRAN_FOUND TRUE)
else(LIBGFORTRAN_LIBRARIES)

  # Try to use gfortran to output its path to search Query gfortran to get the libgfortran.so path
  find_program(_GFORTRAN_EXECUTABLE NAMES gfortran)
  if(_GFORTRAN_EXECUTABLE)
    execute_process(
      COMMAND ${_GFORTRAN_EXECUTABLE} -print-file-name=libgfortran.so
      OUTPUT_VARIABLE _libgfortran_path
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
  endif()
  if(EXISTS ${_libgfortran_path})
    get_filename_component(_libgfortran_path ${_libgfortran_path} DIRECTORY)
  else()
    unset(_libgfortran_path)
  endif()

  find_library(
    GFORTRAN_LIBRARY
    NAMES gfortran
    PATHS ${_libgfortran_path} /usr/lib /usr/local/lib /opt/local/lib /sw/lib
  )

  find_library(
    QUADMATH_LIBRARY
    NAMES quadmath
    PATHS ${_libgfortran_path} /usr/lib /usr/local/lib /opt/local/lib /sw/lib
  )

  if(GFORTRAN_LIBRARY AND QUADMATH_LIBRARY)
    set(LIBGFORTRAN_LIBRARIES ${LIBGFORTRAN_LIBRARIES} ${GFORTRAN_LIBRARY} ${QUADMATH_LIBRARY})
  endif(GFORTRAN_LIBRARY AND QUADMATH_LIBRARY)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(LIBGFORTRAN DEFAULT_MSG GFORTRAN_LIBRARY QUADMATH_LIBRARY)

  # show the LIBGFORTRAN_LIBRARIES variables only in the advanced view
  mark_as_advanced(LIBGFORTRAN_LIBRARIES GFORTRAN_LIBRARY QUADMATH_LIBRARY _GFORTRAN_EXECUTABLE)

endif(LIBGFORTRAN_LIBRARIES)
