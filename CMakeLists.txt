CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
PROJECT(NUbots)

# Set on to prefer .a files over .so files for linking
OPTION(STATIC_LIBRARIES "Prefer using static .a files rather than .so files. NOTE: This must be enabled from the start!" OFF)

IF(STATIC_LIBRARIES)
    SET(CMAKE_FIND_LIBRARY_SUFFIXES
        ${CMAKE_STATIC_LIBRARY_SUFFIX}
        ${CMAKE_SHARED_LIBRARY_SUFFIX})
ENDIF()

# We use additional modules that cmake needs to know about
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")

# Include all of the globally shared link libraries
INCLUDE(SharedLibraries)

# Include specific compiler options
INCLUDE(Compiler)

# Hide some cmake variables that will never be used
MARK_AS_ADVANCED(
    CMAKE_INSTALL_PREFIX
    CURSES_CURSES_H_PATH
    CURSES_FORM_LIBRARY
    CURSES_HAVE_CURSES_H
    OMPLAPP_LIBRARY
    OMPL_CONFIG
    OMPL_DIR
    OMPL_INCLUDE_DIRS
    OMPL_LIBRARIES
    OMPL_LIBRARY
)

# Our banner file for placing at the top of the roles
SET(NUCLEAR_ROLE_BANNER_FILE "${PROJECT_SOURCE_DIR}/cmake/banner.png" CACHE PATH "The path the banner to print at the start of each role execution" FORCE)

# Include the NUClear roles system
ADD_SUBDIRECTORY(nuclear)

# Enable sending binaries to robots
INCLUDE(RobotSend)
