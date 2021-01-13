# Find ALSA for eSpeak
find_package(ALSA REQUIRED)

# Find PortAudio for eSpeak
find_package(PortAudio REQUIRED)

include(ToolchainLibraryFinder)
ToolchainLibraryFinder(
  NAME eSpeak
  HEADER espeak/speak_lib.h
  LIBRARY espeak
)

set(eSpeak_LIBRARIES ${eSpeak_LIBRARIES} ${PortAudio_LIBRARIES} ${ALSA_LIBRARIES})
