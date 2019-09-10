# Default to do a debug build
if(NOT CMAKE_BUILD_TYPE)
  set(
    CMAKE_BUILD_TYPE
    Debug
    CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE
  )
endif()

# RPath variables use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH FALSE)

# Build the RPATH into the binary before install
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# Make OSX use the same RPATH as everyone else
set(CMAKE_MACOSX_RPATH ON)

# Add some useful places to the RPATH These will allow the binary to run from the build folder
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_RPATH} lib/ ../lib/ bin/lib)

if(NOT MSVC)
  # Compilation must be done with c++14 for NUClear to work
  add_compile_options(-std=c++14 -fPIC)
endif()
