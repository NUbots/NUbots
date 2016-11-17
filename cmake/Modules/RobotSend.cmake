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
        
        # Make our installer
        ADD_CUSTOM_TARGET("${host}-${config}"
            USES_TERMINAL
            COMMAND ${PYTHON_EXECUTABLE}
            "${CMAKE_SOURCE_DIR}/nuclear/b.py" "install" "${host}" "--config=${config}" "--user=${user}"
            DEPENDS ${NUCLEAR_ROLES} "${CMAKE_SOURCE_DIR}/tools/install.py")

        # Move our installer to an IDE group
        SET_PROPERTY(TARGET "${host}-${config}" PROPERTY FOLDER "installers")
    ENDFOREACH(config)
ENDFOREACH(host)

ADD_CUSTOM_TARGET("dall"
        DEPENDS d1e-overwrite d2e-overwrite d3e-overwrite d4e-overwrite d5e-overwrite d6e-overwrite i1e-overwrite)
