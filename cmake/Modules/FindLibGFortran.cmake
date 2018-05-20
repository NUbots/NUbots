# - Try to find libGFortran
# Once done this will define
#
#  LIBGFORTRAN_FOUND - system has libGFortran
#  LIBGFORTRAN_LIBRARIES - Link these to use libGFortran
#
#  Copyright (c) 2013 Trent Houliston <trent@houliston.me>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

IF(LIBGFORTRAN_LIBRARIES)
    # in cache already
    SET(LIBGFORTRAN_FOUND TRUE)
ELSE(LIBGFORTRAN_LIBRARIES)
    IF(CMAKE_Fortran_COMPILER)
        EXECUTE_PROCESS(COMMAND ${CMAKE_Fortran_COMPILER} -print-file-name=libgfortran.so
                                OUTPUT_VARIABLE _libgfortran_path
                                OUTPUT_STRIP_TRAILING_WHITESPACE)
        EXECUTE_PROCESS(COMMAND ${CMAKE_Fortran_COMPILER} -print-file-name=libquadmath.so
                                OUTPUT_VARIABLE _libquadmath_path
                                OUTPUT_STRIP_TRAILING_WHITESPACE)
    ELSE()
        # Try to use gfortran to output its path to search
        # Query gfortran to get the libgfortran.so path
        FIND_PROGRAM(_GFORTRAN_EXECUTABLE NAMES gfortran)
        IF(_GFORTRAN_EXECUTABLE)
            EXECUTE_PROCESS(COMMAND ${_GFORTRAN_EXECUTABLE} -print-file-name=libgfortran.so
                                    OUTPUT_VARIABLE _libgfortran_path
                                    OUTPUT_STRIP_TRAILING_WHITESPACE)
            EXECUTE_PROCESS(COMMAND ${_GFORTRAN_EXECUTABLE} -print-file-name=libquadmath.so
                                    OUTPUT_VARIABLE _libquadmath_path
                                    OUTPUT_STRIP_TRAILING_WHITESPACE)
        ENDIF()
    ENDIF()

    IF(EXISTS ${_libgfortran_path})
        GET_FILENAME_COMPONENT(_libgfortran_path ${_libgfortran_path} DIRECTORY)
    ELSE()
        UNSET(_libgfortran_path)
    ENDIF()

    IF(EXISTS ${_libquadmath_path})
        GET_FILENAME_COMPONENT(_libquadmath_path ${_libquadmath_path} DIRECTORY)
    ELSE()
        UNSET(_libquadmath_path)
    ENDIF()

    FIND_LIBRARY(GFORTRAN_LIBRARY
                 NAMES gfortran
                 PATHS ${_libgfortran_path}
    )

    FIND_LIBRARY(QUADMATH_LIBRARY
                 NAMES quadmath
                 PATHS ${_libquadmath_path}
    )

    SET(required_vars GFORTRAN_LIBRARY)

    IF(GFORTRAN_LIBRARY)
        LIST(APPEND LIBGFORTRAN_LIBRARIES ${GFORTRAN_LIBRARY})
    ENDIF(GFORTRAN_LIBRARY)

    IF(QUADMATH_LIBRARY)
        LIST(APPEND LIBGFORTRAN_LIBRARIES ${QUADMATH_LIBRARY})
        LIST(APPEND required_vars QUADMATH_LIBRARY)
    ENDIF(QUADMATH_LIBRARY)

    INCLUDE(FindPackageHandleStandardArgs)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(LIBGFORTRAN DEFAULT_MSG ${required_vars})

    # show the LIBGFORTRAN_LIBRARIES variables only in the advanced view
    MARK_AS_ADVANCED(LIBGFORTRAN_LIBRARIES GFORTRAN_LIBRARY QUADMATH_LIBRARY _GFORTRAN_EXECUTABLE)
ENDIF(LIBGFORTRAN_LIBRARIES)
