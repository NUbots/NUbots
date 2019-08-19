# We need python!
find_package(PythonInterp REQUIRED)

# Script output is like d1;darwin1 d2;darwin2
execute_process(COMMAND "/nubots/toolchain/find_robot_hosts.sh" OUTPUT_VARIABLE HOSTS)

# Convert script output to a list of pairs.
separate_arguments(KNOWN_HOSTS UNIX_COMMAND "${HOSTS}")

set(user "nubots")

# Ninja code!
foreach(host_pair ${KNOWN_HOSTS})
  # Get each element of the pair. host  = d1 alias = darwin1
  list(GET host_pair 0 host)
  list(GET host_pair 1 alias)

  foreach(
    config
    ""
    n
    u
    o
    i
    t
  )
    if(config STREQUAL "")
      set(target "${host}")
    else()
      set(target "${host}${config}")
    endif()

    # Make our installer The install script expects an IP address to install to and a hostname to determine config files
    # to install. IP address is represented by the short-form hostname (e.g. d1) Hostname is represented by the long-
    # form hostname (e.g. darwin1)
    if(config STREQUAL "t")
      add_custom_target(
        "${target}" USES_TERMINAL
        COMMAND
          ${PYTHON_EXECUTABLE}
          "${CMAKE_SOURCE_DIR}/nuclear/b.py"
          "install"
          "${host}"
          "${alias}"
          "--user=${user}"
          "--toolchain"
        DEPENDS ${NUCLEAR_ROLES} "${CMAKE_SOURCE_DIR}/tools/install.py"
      )
    else()
      add_custom_target(
        "${target}" USES_TERMINAL
        COMMAND
          ${PYTHON_EXECUTABLE}
          "${CMAKE_SOURCE_DIR}/nuclear/b.py"
          "install"
          "${host}"
          "${alias}"
          "--config=${config}"
          "--user=${user}"
        DEPENDS ${NUCLEAR_ROLES} "${CMAKE_SOURCE_DIR}/tools/install.py"
      )
    endif()

    # Move our installer to an IDE group
    set_property(TARGET "${target}" PROPERTY FOLDER "installers")
  endforeach(config)
endforeach(host_pair)
