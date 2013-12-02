# Custom target to copy and install configuration files and binarys to the robot
ADD_CUSTOM_TARGET(install 
                  COMMAND bash "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.sh" \${robot} \${config}
                  DEPENDS ${roles} "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.sh")

# Custom target to copy and install configuration files and binarys to the robot
ADD_CUSTOM_TARGET(run 
                  COMMAND bash "${CMAKE_SOURCE_DIR}/cmake/Scripts/run.sh" \${robot} \${config}
                  DEPENDS install "${CMAKE_SOURCE_DIR}/cmake/Scripts/run.sh")
