INCLUDE("${CMAKE_SOURCE_DIR}/cmake/Includes/AddRole.cmake")

# If you're using ConfigSystem then it needs to be the first installed module.
# It needs to be installed first because other modules are going to try and
# request configurations from it on install and if it hasn't been installed yet
# they won't get any data and things will break.

# This gets all of our source files
FILE(GLOB rolefiles "*.role")

# Find all of our role files
FOREACH(rolefile ${rolefiles})
    GET_FILENAME_COMPONENT(roleext ${rolefile} EXT)

    # Check it is a .role file
    IF(roleext STREQUAL .role)

        # Get role name and add it to our list of roles
        GET_FILENAME_COMPONENT(role_name ${rolefile} NAME_WE)
        SET(base_roles ${base_roles} ${role_name})
    ENDIF()
ENDFOREACH(rolefile)

# Set the option to build only some of the roles
SET(BUILD_ROLES "${base_roles}" CACHE STRING "A list of semicolon separated roles that are to be built by the system")

# If we have explicit roles to build build them
IF(BUILD_ROLES)
    set(roles ${BUILD_ROLES})
ELSE()
    set(roles ${base_roles})
ENDIF()

FOREACH(role ${roles})
    INCLUDE("${role}.role")
ENDFOREACH(role)

# The ADD_ROLE function appends itself to the ${roles} variable.
# We need to expose this to our parent scope so the DarwinSend.cmake
# file knows what binaries it depends on. We do this so that "make install"
# and "make run" will properly update when binaries are regenerated.
SET(roles ${roles} PARENT_SCOPE)
SET(modules ${modules} PARENT_SCOPE)
