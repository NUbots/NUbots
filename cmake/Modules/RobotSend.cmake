# We need python!
FIND_PACKAGE(PythonInterp REQUIRED)

# Script output is like
# d1;darwin1
# d2;darwin2
EXECUTE_PROCESS(COMMAND "/nubots/toolchain/find_robot_hosts.sh" OUTPUT_VARIABLE HOSTS)

# Convert script output to a list of pairs.
SEPARATE_ARGUMENTS(KNOWN_HOSTS UNIX_COMMAND "${HOSTS}")

SET(user "nubots")

# Ninja code!
FOREACH(host_pair ${KNOWN_HOSTS})
    # Get each element of the pair.
    # host  = d1
    # alias = darwin1
    LIST(GET host_pair 0 host)
    LIST(GET host_pair 1 alias)

    FOREACH(config "" n u o i t)
        IF (config STREQUAL "")
            SET(target "${host}")
        ELSE()
            SET(target "${host}${config}")
        ENDIF()
        
        # Make our installer
        # The install script expects an IP address to install to and a hostname to determine config files to install.
        # IP address is represented by the short-form hostname (e.g. d1)
        # Hostname is represented by the long-form hostname (e.g. darwin1)
        IF (config STREQUAL "t")
            ADD_CUSTOM_TARGET("${target}"
                USES_TERMINAL
                COMMAND ${PYTHON_EXECUTABLE}
                "${CMAKE_SOURCE_DIR}/nuclear/b.py" "install" "${host}" "${alias}" "--user=${user}" "--toolchain"
                DEPENDS ${NUCLEAR_ROLES} "${CMAKE_SOURCE_DIR}/tools/install.py")
        ELSE()
            ADD_CUSTOM_TARGET("${target}"
                USES_TERMINAL
                COMMAND ${PYTHON_EXECUTABLE}
                "${CMAKE_SOURCE_DIR}/nuclear/b.py" "install" "${host}" "${alias}" "--config=${config}" "--user=${user}"
                DEPENDS ${NUCLEAR_ROLES} "${CMAKE_SOURCE_DIR}/tools/install.py")
        ENDIF()

        # Move our installer to an IDE group
        SET_PROPERTY(TARGET "${target}" PROPERTY FOLDER "installers")
    ENDFOREACH(config)
ENDFOREACH(host_pair)
