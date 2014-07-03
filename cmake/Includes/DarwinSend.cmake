# Custom target to copy and install configuration files and binarys to the robot
ADD_CUSTOM_TARGET(install
                  COMMAND python "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py" "--robot_ip=\${robot}" "--config=\${config}" "--username=\${username}"
                  DEPENDS ${roles} "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py")

# Ninja code!
FOREACH(robot 1 2 3 4 5 6 7)
    FOREACH(config "" u o n i)

        # Make our installer
        ADD_CUSTOM_TARGET("d${robot}${config}"
                          COMMAND python "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py" "--robot_ip=10.0.1.5${robot}" "--config=${config}" "--username=`whoami`"
                          DEPENDS ${roles} "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py")


        # Move our installer to an IDE group
        SET_PROPERTY(TARGET "d${robot}${config}" PROPERTY FOLDER "installers")

    ENDFOREACH(config)
ENDFOREACH(robot)

