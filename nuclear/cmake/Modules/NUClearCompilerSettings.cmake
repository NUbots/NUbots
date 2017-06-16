# Default to do a debug build
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Debug CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
       FORCE)
ENDIF()

# RPath variables
# use, i.e. don't skip the full RPATH for the build tree
SET(CMAKE_SKIP_BUILD_RPATH FALSE)

# Build the RPATH into the binary before install
SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# Make OSX use the same RPATH as everyone else
SET(CMAKE_MACOSX_RPATH ON)

# Add some useful places to the RPATH
# These will allow the binary to run from the build folder
SET(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_RPATH} lib/ ../lib/ bin/lib)

IF(NOT MSVC)
    # Compilation must be done with c++14 for NUClear to work
    ADD_COMPILE_OPTIONS(-std=c++14 -fPIC)
ENDIF()
