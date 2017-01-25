# We need python!
FIND_PACKAGE(PythonInterp REQUIRED)

EXECUTE_PROCESS(COMMAND "/nubots/toolchain/find_robot_hosts.sh" OUTPUT_VARIABLE HOSTS)
SEPARATE_ARGUMENTS(KNOWN_HOSTS UNIX_COMMAND "${HOSTS}")

# Ninja code!
FOREACH(host ${KNOWN_HOSTS})
    FOREACH(config "" new update overwrite pull ignore)
        IF ("${host}" MATCHES "i[0-9]+")
            SET(user "nubots")
        ELSE()
            SET(user "darwin")
        ENDIF()

        IF (config MATCHES "[a-z]+")
            SET(target "${host}-${config}")
        ELSE()
            SET(target "${host}")
        ENDIF()
        
        # Make our installer
        # The install script expects an IP address to install to and a hostname to determine config files to install.
        # Our build host has defined hostname/IP addresses for install targets so we can just specify the host twice.
        ADD_CUSTOM_TARGET("${target}"
            USES_TERMINAL
            COMMAND ${PYTHON_EXECUTABLE}
            "${CMAKE_SOURCE_DIR}/nuclear/b.py" "install" "${host}" "${host}" "--config=${config}" "--user=${user}"
            DEPENDS ${NUCLEAR_ROLES} "${CMAKE_SOURCE_DIR}/tools/install.py")

        # Move our installer to an IDE group
        SET_PROPERTY(TARGET "${target}" PROPERTY FOLDER "installers")
    ENDFOREACH(config)
ENDFOREACH(host)

ADD_CUSTOM_TARGET("dall"
        DEPENDS d1e-overwrite d2e-overwrite d3e-overwrite d4e-overwrite d5e-overwrite d6e-overwrite i1e-overwrite)
