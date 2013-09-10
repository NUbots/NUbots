INCLUDE("${CMAKE_SOURCE_DIR}/cmake/Includes/AddHat.cmake")

# If you're using ConfigSystem then it needs to be the first installed module.
# It needs to be installed first because other modules are going to try and
# request configurations from it on install and if it hasn't been installed yet
# they won't get any data and things will break.
ADD_HAT(
    NAME robocup
    MODULES 
        ConfigSystem
        DarwinPlatform
        DarwinCameraReader
        DarwinMotionManager
        ScriptEngine
        eSpeak
        PartyDarwin
        AudioInput
        NUBugger)
        

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
