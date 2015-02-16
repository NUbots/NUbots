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

# If we don't have any roles then default to all roles
IF(NOT NUCLEAR_ROLES)
    SET(NUCLEAR_ROLES "${base_roles}" CACHE STRING "A list of semicolon separated roles that are to be built by the system" FORCE)
ENDIF()

# Note that this sets the local role variable that is used in the included role file
FOREACH(role ${NUCLEAR_ROLES})
    INCLUDE("${role}.role")
ENDFOREACH(role)
