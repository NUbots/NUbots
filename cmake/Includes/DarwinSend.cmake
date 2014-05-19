# Custom target to copy and install configuration files and binarys to the robot
ADD_CUSTOM_TARGET(install
                  COMMAND python "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py" "--robot_ip=\${robot}" "--config=\${config}" "--username=\${username}"
                  DEPENDS ${roles} "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py")

# Custom target to copy and install configuration files and binarys to the robot
ADD_CUSTOM_TARGET(run
                  COMMAND bash "${CMAKE_SOURCE_DIR}/cmake/Scripts/run.sh" \${robot} \${config}
                  DEPENDS install "${CMAKE_SOURCE_DIR}/cmake/Scripts/run.sh")
