# Custom target to copy and install configuration files and binarys to the robot
ADD_CUSTOM_TARGET(install 
                  COMMAND bash "${NUBOTS_SCRIPTS_DIR}/install.sh" \${robot} \${config}
                  DEPENDS ${roles} "${NUBOTS_SCRIPTS_DIR}/install.sh")

# Custom target to copy and install configuration files and binarys to the robot
ADD_CUSTOM_TARGET(run 
                  COMMAND bash "${NUBOTS_SCRIPTS_DIR}/run.sh" \${robot} \${config}
                  DEPENDS install "${NUBOTS_SCRIPTS_DIR}/run.sh")
