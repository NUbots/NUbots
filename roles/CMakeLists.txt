INCLUDE("${CMAKE_SOURCE_DIR}/cmake/Includes/AddRole.cmake")

# If you're using ConfigSystem then it needs to be the first installed module.
# It needs to be installed first because other modules are going to try and
# request configurations from it on install and if it hasn't been installed yet
# they won't get any data and things will break.

# This gets all of our source files
FILE(GLOB roledefs "*.role")

FOREACH(roledef ${roledefs}) 
    INCLUDE(${roledef}) 
ENDFOREACH(roledef) 

# The ADD_HAT function appends itself to the ${hats} variable.
# We need to expose this to our parent scope so the DarwinSend.cmake
# file knows what binaries it depends on. We do this so that "make install"
# and "make run" will properly update when binaries are regenerated.
SET(hats ${hats} PARENT_SCOPE)
SET(modules ${modules} PARENT_SCOPE)
