INCLUDE("${CMAKE_SOURCE_DIR}/cmake/Includes/AddRole.cmake")

# If you're using ConfigSystem then it needs to be the first installed module.
# It needs to be installed first because other modules are going to try and
# request configurations from it on install and if it hasn't been installed yet
# they won't get any data and things will break.
ADD_ROLE(
    NAME robocup
    MODULES
        SignalCatcher
        ConfigSystem
        Platform::Darwin::HardwareIO
        Platform::Darwin::MotionManager
        LinuxCameraStreamer
        ScriptEngine
        eSpeak
        PartyDarwin
        AudioInput
        NUbugger)

ADD_ROLE(
    NAME scriptrunner
    MODULES
        ConfigSystem
        Platform::Darwin::HardwareIO
        Platform::Darwin::MotionManager
        ScriptEngine
        ScriptRunner
        NUbugger
        PartyDarwin)

ADD_ROLE(
    NAME scripttuner
    MODULES
        Platform::Darwin::HardwareIO
        Platform::Darwin::MotionManager
        NUbugger
        ScriptTuner
        PartyDarwin)

ADD_ROLE(
    NAME mechwarrior
    MODULES
        Platform::Darwin::HardwareIO
        MechWarrior)

ADD_ROLE(
    NAME dancedarwin
    MODULES
        ConfigSystem
        Platform::Darwin::HardwareIO
        Platform::Darwin::MotionManager
        AudioFileInput
        AubioBeatDetector
        ScriptEngine
        DanceEngine
        StrobeDarwin)

# The ADD_ROLE function appends itself to the ${roles} variable.
# We need to expose this to our parent scope so the DarwinSend.cmake
# file knows what binaries it depends on. We do this so that "make install"
# and "make run" will properly update when binaries are regenerated.
SET(roles ${roles} PARENT_SCOPE)
SET(modules ${modules} PARENT_SCOPE)
