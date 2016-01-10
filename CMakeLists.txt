CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(NUbots)

# Set on to prefer .a files over .so files for linking
OPTION(STATIC_LIBRARIES "Prefer using static .a files rather than .so files. NOTE: This must be enabled from the start!" OFF)

IF(STATIC_LIBRARIES)
    SET(CMAKE_FIND_LIBRARY_SUFFIXES
        ${CMAKE_STATIC_LIBRARY_SUFFIX}
        ${CMAKE_SHARED_LIBRARY_SUFFIX})
ENDIF()

# We use additional modules that cmake needs to know about
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

# Include all of the globally shared link_libraries
INCLUDE(SharedLibraries)

# Include the NUClear roles system
ADD_SUBDIRECTORY(nuclear)