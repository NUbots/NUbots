# Custom target to copy and install configuration files and binarys to the robot
ADD_CUSTOM_TARGET(install
                  COMMAND python "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py" "--robot_ip=\${robot}" "--config=\${config}" "--username=\${username}"
                  DEPENDS ${roles} "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py")

# Custom target to copy and install configuration files and binarys to the robot
ADD_CUSTOM_TARGET(d1
                  COMMAND python "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py" "--robot_ip=10.0.1.51" "--config=u"
                  DEPENDS ${roles} "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py")
# Custom target to copy and install configuration files and binarys to the robot
ADD_CUSTOM_TARGET(d2
                  COMMAND python "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py" "--robot_ip=10.0.1.52" "--config=u"
                  DEPENDS ${roles} "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py")
# Custom target to copy and install configuration files and binarys to the robot
ADD_CUSTOM_TARGET(d3
                  COMMAND python "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py" "--robot_ip=10.0.1.53" "--config=u"
                  DEPENDS ${roles} "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py")
# Custom target to copy and install configuration files and binarys to the robot
ADD_CUSTOM_TARGET(d4
                  COMMAND python "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py" "--robot_ip=10.0.1.54" "--config=u"
                  DEPENDS ${roles} "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py")
# Custom target to copy and install configuration files and binarys to the robot
ADD_CUSTOM_TARGET(d5
                  COMMAND python "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py" "--robot_ip=10.0.1.55" "--config=u"
                  DEPENDS ${roles} "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py")
# Custom target to copy and install configuration files and binarys to the robot
ADD_CUSTOM_TARGET(d6
                  COMMAND python "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py" "--robot_ip=10.0.1.56" "--config=u"
                  DEPENDS ${roles} "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py")
# Custom target to copy and install configuration files and binarys to the robot
ADD_CUSTOM_TARGET(d7
                  COMMAND python "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py" "--robot_ip=10.0.1.57" "--config=u"
                  DEPENDS ${roles} "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py")

# Custom target to copy and install configuration files and binarys to the robot
ADD_CUSTOM_TARGET(d1o
                  COMMAND python "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py" "--robot_ip=10.0.1.51" "--config=o"
                  DEPENDS ${roles} "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py")
# Custom target to copy and install configuration files and binarys to the robot
ADD_CUSTOM_TARGET(d2o
                  COMMAND python "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py" "--robot_ip=10.0.1.52" "--config=o"
                  DEPENDS ${roles} "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py")
# Custom target to copy and install configuration files and binarys to the robot
ADD_CUSTOM_TARGET(d3o
                  COMMAND python "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py" "--robot_ip=10.0.1.53" "--config=o"
                  DEPENDS ${roles} "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py")
# Custom target to copy and install configuration files and binarys to the robot
ADD_CUSTOM_TARGET(d4o
                  COMMAND python "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py" "--robot_ip=10.0.1.54" "--config=o"
                  DEPENDS ${roles} "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py")
# Custom target to copy and install configuration files and binarys to the robot
ADD_CUSTOM_TARGET(d5o
                  COMMAND python "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py" "--robot_ip=10.0.1.55" "--config=o"
                  DEPENDS ${roles} "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py")
# Custom target to copy and install configuration files and binarys to the robot
ADD_CUSTOM_TARGET(d6o
                  COMMAND python "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py" "--robot_ip=10.0.1.56" "--config=o"
                  DEPENDS ${roles} "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py")
# Custom target to copy and install configuration files and binarys to the robot
ADD_CUSTOM_TARGET(d7o
                  COMMAND python "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py" "--robot_ip=10.0.1.57" "--config=o"
                  DEPENDS ${roles} "${CMAKE_SOURCE_DIR}/cmake/Scripts/install.py")


