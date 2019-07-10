# Find ALSA for eSpeak
FIND_PACKAGE(ALSA REQUIRED)

# Find PortAudio for eSpeak
FIND_PACKAGE(PortAudio REQUIRED)

INCLUDE(ToolchainLibraryFinder)
ToolchainLibraryFinder(NAME eSpeak
                       HEADER espeak/speak_lib.h
                       LIBRARY espeak
                       #BINARY espeak
                       #VERSION_BINARY_ARGUMENTS "--version"
                       #VERSION_REGEX "eSpeak text-to-speech: (([0-9]+\\.?)+)"
)

SET(eSpeak_LIBRARIES ${eSpeak_LIBRARIES} ${PortAudio_LIBRARIES} ${ALSA_LIBRARIES})
