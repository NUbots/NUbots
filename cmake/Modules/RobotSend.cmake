# We need python!
FIND_PACKAGE(PythonInterp REQUIRED)

# Script output is like
# d1;darwin1
# d2;darwin2
EXECUTE_PROCESS(COMMAND "/nubots/toolchain/find_robot_hosts.sh" OUTPUT_VARIABLE HOSTS)

# Convert script output to a list of pairs.
SEPARATE_ARGUMENTS(KNOWN_HOSTS UNIX_COMMAND "${HOSTS}")

# Ninja code!
FOREACH(host_pair ${KNOWN_HOSTS})
    # Get each element of the pair.
    # host  = d1
    # alias = darwin1
    LIST(GET host_pair 0 host)
    LIST(GET host_pair 1 alias)

    IF ("${host}" MATCHES "i[0-9]+")
        SET(user "nubots")
    ELSE()
        SET(user "darwin")
    ENDIF()

    FOREACH(config "" n u o i p)
        FOREACH(scripts "" n u o i p)
            IF ((config STREQUAL "") AND (scripts STREQUAL ""))
                SET(target "${host}")
            ELSE()
                SET(target "${host}-c${config}-s${scripts}")
            ENDIF()
            
            # Make our installer
            # The install script expects an IP address to install to and a hostname to determine config files to install.
            # IP address is represented by the short-form hostname (e.g. d1)
            # Hostname is represented by the long-form hostname (e.g. darwin1)
            ADD_CUSTOM_TARGET("${target}"
                USES_TERMINAL
                COMMAND ${PYTHON_EXECUTABLE}
                "${CMAKE_SOURCE_DIR}/nuclear/b.py" "install" "${host}" "${alias}" "--config=${config}" "--scripts=${scripts}" "--user=${user}"
                DEPENDS ${NUCLEAR_ROLES} "${CMAKE_SOURCE_DIR}/tools/install.py")

            # Move our installer to an IDE group
            SET_PROPERTY(TARGET "${target}" PROPERTY FOLDER "installers")
        ENDFOREACH(scripts)
    ENDFOREACH(config)
ENDFOREACH(host_pair)

ADD_CUSTOM_TARGET("dall" DEPENDS d1-co-so d2-co-so d3-co-so d4-co-so d5-co-so d6-co-so i1-co-so)
