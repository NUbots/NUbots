# Custom target to copy and install configuration files and binarys to the robot
ADD_CUSTOM_TARGET(install 
                  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/install.sh ${CMAKE_BINARY_DIR}/install.sh
                  COMMAND bash "${NUBOTS_SCRIPTS_DIR}/install.sh" \${robot} \${config}
                  DEPENDS ${hats})

# Custom target to copy and install configuration files and binarys to the robot
ADD_CUSTOM_TARGET(run 
                  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/run.sh ${CMAKE_BINARY_DIR}/run.sh
                  COMMAND bash "${NUBOTS_SCRIPTS_DIR}/run.sh" \${robot} \${config}
                  DEPENDS install)
