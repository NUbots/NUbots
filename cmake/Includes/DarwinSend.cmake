MESSAGE("${CMAKE_COMMAND}")

ADD_CUSTOM_TARGET(install scp robocup darwin@\${robot}:/home/darwin/robocup
                  DEPENDS robocup)