# - Find librt
# Find the native LIBRT includes and library
#
#  LIBRT_INCLUDE_DIR - where to find time.h, etc.
#  LIBRT_LIBRARIES   - List of libraries when using librt.
#  LIBRT_FOUND       - True if librt found.

MESSAGE(STATUS "Running the FindLIBRT.cmake script")

IF (LIBRT_INCLUDE_DIR)
  # Already in cache, be silent
  SET(LIBRT_FIND_QUIETLY TRUE)
ENDIF (LIBRT_INCLUDE_DIR)

FIND_PATH(LIBRT_INCLUDE_DIR time.h)

SET(LIBRT_NAMES rt librt)
FIND_LIBRARY(LIBRT_LIBRARY NAMES ${LIBRT_NAMES} )

# handle the QUIETLY and REQUIRED arguments and set ZLIB_FOUND to TRUE if 
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(LIBRT DEFAULT_MSG LIBRT_LIBRARY LIBRT_INCLUDE_DIR)

IF(LIBRT_FOUND)
  SET( LIBRT_LIBRARIES ${LIBRT_LIBRARY} )
ELSE(LIBRT_FOUND)
  SET( LIBRT_LIBRARIES )
ENDIF(LIBRT_FOUND)

MARK_AS_ADVANCED( LIBRT_LIBRARY LIBRT_INCLUDE_DIR )
