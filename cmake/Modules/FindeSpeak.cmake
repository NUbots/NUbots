# Find ALSA for eSpeak
find_package(ALSA REQUIRED)

# Find PortAudio for eSpeak
find_package(PortAudio REQUIRED)

include(ToolchainLibraryFinder)
ToolchainLibraryFinder(
  NAME eSpeak
  HEADER espeak/speak_lib.h
  LIBRARY
    espeak
    # BINARY espeak
    # VERSION_BINARY_ARGUMENTS "--version"
    # VERSION_REGEX "eSpeak text-to-speech: (([0-9]+\\.?)+)"
)

set(eSpeak_LIBRARIES ${eSpeak_LIBRARIES} ${PortAudio_LIBRARIES} ${ALSA_LIBRARIES})
