# - Find the Pthreads library
# This module searches for the Pthreads library (including the
# pthreads-win32 port).
#
# This module defines these variables:
#
#  PTHREAD_FOUND
#      True if the Pthreads library was found
#  PTHREAD_LIBRARIES
#      The location of the Pthreads library
#  PTHREAD_INCLUDE_DIR
#      The include path of the Pthreads library
#  PTHREAD_DEFINITIONS
#      Preprocessor definitions to define
#
# This module responds to the PTHREAD_EXCEPTION_SCHEME
# variable on Win32 to allow the user to control the
# library linked against.  The Pthreads-win32 port
# provides the ability to link against a version of the
# library with exception handling.  IT IS NOT RECOMMENDED
# THAT YOU USE THIS because most POSIX thread implementations
# do not support stack unwinding.
#
#  PTHREAD_EXCEPTION_SCHEME
#      C  = no exceptions (default)
#         (NOTE: This is the default scheme on most POSIX thread
#          implementations and what you should probably be using)
#      CE = C++ Exception Handling
#      SE = Structure Exception Handling (MSVC only)
#

#
# Define a default exception scheme to link against
# and validate user choice.
#
IF(NOT DEFINED PTHREAD_EXCEPTION_SCHEME)
    # Assign default if needed
    SET(PTHREAD_EXCEPTION_SCHEME "C")
ELSE(NOT DEFINED PTHREAD_EXCEPTION_SCHEME)
    # Validate
    IF(NOT PTHREAD_EXCEPTION_SCHEME STREQUAL "C" AND
       NOT PTHREAD_EXCEPTION_SCHEME STREQUAL "CE" AND
       NOT PTHREAD_EXCEPTION_SCHEME STREQUAL "SE")

    MESSAGE(FATAL_ERROR "See documentation for FindPthreads.cmake, only C, CE, and SE modes are allowed")

    ENDIF(NOT PTHREAD_EXCEPTION_SCHEME STREQUAL "C" AND
          NOT PTHREAD_EXCEPTION_SCHEME STREQUAL "CE" AND
          NOT PTHREAD_EXCEPTION_SCHEME STREQUAL "SE")

     IF(NOT MSVC AND PTHREAD_EXCEPTION_SCHEME STREQUAL "SE")
         MESSAGE(FATAL_ERROR "Structured Exception Handling is only allowed for MSVC")
     ENDIF(NOT MSVC AND PTHREAD_EXCEPTION_SCHEME STREQUAL "SE")

ENDIF(NOT DEFINED PTHREAD_EXCEPTION_SCHEME)

#
# Find the header file
#
FIND_PATH(PTHREAD_INCLUDE_DIR pthread.h)

#
# Find the library
#
SET(names)
IF(MSVC)
    SET(names
            pthreadV${PTHREAD_EXCEPTION_SCHEME}2
            pthread
    )
ELSEIF(MINGW)
    SET(names
            pthreadG${PTHREAD_EXCEPTION_SCHEME}2
            pthread
    )
ELSE(MSVC) # Unix / Cygwin / Apple
    SET(names pthread)
ENDIF(MSVC)
    
FIND_LIBRARY(PTHREAD_LIBRARY ${names}
    DOC "The Portable Threads Library")

IF(PTHREAD_INCLUDE_DIR AND PTHREAD_LIBRARY)
    SET(PTHREAD_FOUND true)
    SET(PTHREAD_DEFINITIONS -DHAVE_PTHREAD_H)
    SET(PTHREAD_INCLUDE_DIRS ${PTHREAD_INCLUDE_DIR})
    SET(PTHREAD_LIBRARIES    ${PTHREAD_LIBRARY})
ENDIF(PTHREAD_INCLUDE_DIR AND PTHREAD_LIBRARY)

IF(PTHREAD_FOUND)
    IF(NOT PTHREAD_FIND_QUIETLY)
        MESSAGE(STATUS "Found Pthreads: ${PTHREAD_LIBRARY}")
    ENDIF(NOT PTHREAD_FIND_QUIETLY)
ELSE(PTHREAD_FOUND) 
    IF(PTHREAD_FIND_REQUIRED)
        MESSAGE(FATAL_ERROR "Could not find the Pthreads Library")
    ENDIF(PTHREAD_FIND_REQUIRED)
ENDIF(PTHREAD_FOUND)

MARK_AS_ADVANCED(
    PTHREAD_DIR
    PTHREAD_LIBRARY
    PTHREAD_INCLUDE_DIR
)
