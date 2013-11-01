INCLUDE("${CMAKE_SOURCE_DIR}/cmake/Includes/AddHat.cmake")

# If you're using ConfigSystem then it needs to be the first installed module.
# It needs to be installed first because other modules are going to try and
# request configurations from it on install and if it hasn't been installed yet
# they won't get any data and things will break.
ADD_HAT(
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
        NUBugger)

ADD_HAT(
    NAME scriptrunner
    MODULES
        ConfigSystem
        Platform::Darwin::HardwareIO
        Platform::Darwin::MotionManager
        ScriptEngine
        ScriptRunner
        NUBugger
        PartyDarwin)

ADD_HAT(
    NAME scripttuner
    MODULES
        Platform::Darwin::HardwareIO
        Platform::Darwin::MotionManager
        NUBugger
        ScriptTuner
        PartyDarwin)

ADD_HAT(
    NAME mechwarrior
    MODULES
        Platform::Darwin::HardwareIO
        MechWarrior)

ADD_HAT(
    NAME dancedarwin
    MODULES
        ConfigSystem
        Platform::Darwin::HardwareIO
        Platform::Darwin::MotionManager
        AudioFileInput
        AubioBeatDetector
        ScriptEngine
        DanceEngine)

# The ADD_HAT function appends itself to the ${hats} variable.
# We need to expose this to our parent scope so the DarwinSend.cmake
# file knows what binaries it depends on. We do this so that "make install"
# and "make run" will properly update when binaries are regenerated.
SET(hats ${hats} PARENT_SCOPE)
SET(modules ${modules} PARENT_SCOPE)
