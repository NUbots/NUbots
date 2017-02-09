INCLUDE(AddRole)

# If you're using ConfigSystem then it needs to be the first installed module.
# It needs to be installed first because other modules are going to try and
# request configurations from it on install and if it hasn't been installed yet
# they won't get any data and things will break.

# This gets all of our source files
FILE(GLOB rolefiles "*.role")

# Backup all the current role variables (so we can reuse them)
# and delete them
GET_CMAKE_PROPERTY(vars CACHE_VARIABLES)
FOREACH(var ${vars})
    IF(var MATCHES "^ROLE_[a-zA-Z0-9]+")
        SET("BACKUP_${var}" ${${var}})
        UNSET(${var} CACHE)
    ENDIF()
ENDFOREACH(var)

# Find all of our role files
FOREACH(rolefile ${rolefiles})
    GET_FILENAME_COMPONENT(roleext ${rolefile} EXT)

    # Check it is a .role file
    IF(roleext STREQUAL .role)

        # Get role name and add it to our list of roles
        GET_FILENAME_COMPONENT(role_name ${rolefile} NAME_WE)

        # If we had a backup, use it otherwise default to true
        IF(DEFINED "BACKUP_ROLE_${role_name}")
            OPTION("ROLE_${role_name}" "Build the role ${role_name}" ${BACKUP_ROLE_${role_name}})
        ELSE()
            OPTION("ROLE_${role_name}" "Build the role ${role_name}" ON)
        ENDIF()
    ENDIF()
ENDFOREACH(rolefile)

# Set our NUClear roles based on the ROLES cache variables that are set to true
GET_CMAKE_PROPERTY(vars CACHE_VARIABLES)
UNSET(NUCLEAR_ROLES CACHE)
FOREACH(var ${vars})
    IF(var MATCHES "^ROLE_[a-zA-Z0-9]+")
        IF(${var})
            STRING(SUBSTRING ${var} 5  -1 role_name)
            SET(NUCLEAR_ROLES ${NUCLEAR_ROLES} ${role_name} CACHE INTERNAL "A list of the modules in use by the system" FORCE)
        ENDIF()
    ENDIF()
ENDFOREACH()

# Note that this sets the local role variable that is used in the included role file
FOREACH(role ${NUCLEAR_ROLES})
    INCLUDE("${role}.role")
ENDFOREACH(role)
