# We need python!
FIND_PACKAGE(PythonInterp REQUIRED)

# Custom target to copy and install configuration files and binarys to the robot
ADD_CUSTOM_TARGET(send
                  COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_SOURCE_DIR}/cmake/Scripts/send.py" "--robot_ip=\${robot}" "--config=\${config}" "--username=\${username}"
                  DEPENDS ${roles} "${CMAKE_SOURCE_DIR}/cmake/Scripts/send.py")

# Put it in an IDE group for shared
SET_PROPERTY(TARGET send PROPERTY FOLDER "ALL_BUILD/")

# Ninja code!
FOREACH(robot 1 2 3 4 5 6 7)
    FOREACH(config "" u o n i)
        FOREACH(ethernet "" e)
            IF("${ethernet}" STREQUAL "e")
                SET(address "10.1.2.${robot}")
            ELSE()
                SET(address "10.1.1.${robot}")
            ENDIF()

            # Make our installer
            ADD_CUSTOM_TARGET("d${robot}${ethernet}${config}"
                COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_SOURCE_DIR}/cmake/Scripts/send.py" "--robot_ip=${address}" "--config=${config}" "--username=darwin"
                DEPENDS ${roles} "${CMAKE_SOURCE_DIR}/cmake/Scripts/send.py")


            # Move our installer to an IDE group
            SET_PROPERTY(TARGET "d${robot}${ethernet}${config}" PROPERTY FOLDER "installers")

        ENDFOREACH(ethernet)
    ENDFOREACH(config)
ENDFOREACH(robot)

ADD_CUSTOM_TARGET("nc"
          COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_SOURCE_DIR}/cmake/Scripts/send.py" "--robot_ip=10.0.1.120" "--username=nubots"
          DEPENDS ${roles} "${CMAKE_SOURCE_DIR}/cmake/Scripts/send.py")
