INCLUDE("${CMAKE_SOURCE_DIR}/cmake/Includes/AddHat.cmake")

ADD_EXECUTABLE(scripttuner scripttuner.cpp)
TARGET_LINK_LIBRARIES(scripttuner
    ${NUBOTS_SHARED_LIBRARIES}
    ${modules})

ADD_EXECUTABLE(scriptrunner scriptrunner.cpp)
TARGET_LINK_LIBRARIES(scriptrunner
    ${NUBOTS_SHARED_LIBRARIES}
    ${modules})

SET(hats
    scripttuner
    scriptrunner)
