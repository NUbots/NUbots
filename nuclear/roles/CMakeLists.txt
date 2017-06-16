# Get our role directories
SET(role_source_dir "${PROJECT_SOURCE_DIR}/roles")
SET(role_binary_dir "${PROJECT_BINARY_DIR}/role")

# This gets all of our source files
FILE(GLOB rolefiles "${role_source_dir}/*.role")

# Backup the state of all the role variables before we re read them
# This is needed so we can delete roles that no longer exist
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

        # If we had a backup as to if this role is enabled use it.
        # Otherwise default to true.
        IF(DEFINED "BACKUP_ROLE_${role_name}")
            OPTION("ROLE_${role_name}" "Build the role ${role_name}" ${BACKUP_ROLE_${role_name}})
        ELSE()
            OPTION("ROLE_${role_name}" "Build the role ${role_name}" ON)
        ENDIF()
    ENDIF()
ENDFOREACH(rolefile)

# Set our NUClear roles based on the ROLES cache variables that are set to true
# This makes it so that only the roles that are enabled will be built
GET_CMAKE_PROPERTY(vars CACHE_VARIABLES)
UNSET(NUCLEAR_ROLES CACHE)
FOREACH(var ${vars})
    IF(var MATCHES "^ROLE_[a-zA-Z0-9]+")
        IF(${var})
            STRING(SUBSTRING ${var} 5  -1 role_name)
            SET(NUCLEAR_ROLES ${NUCLEAR_ROLES} ${role_name} CACHE INTERNAL "A list of the roles that are enabled to be built." FORCE)
        ENDIF()
    ENDIF()
ENDFOREACH()

# We need the NUClear role function here so it can be used in the role files
INCLUDE(NUClearRole.cmake)

# Clear our NUClear modules variable so we can repopulate it
UNSET(NUCLEAR_MODULES CACHE)

# Note that the loop value of this for loop sets the local role variable that is used in the included role file
FOREACH(role ${NUCLEAR_ROLES})
    INCLUDE("${role_source_dir}/${role}.role")
ENDFOREACH(role)
