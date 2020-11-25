# Find ALSA for eSpeak
include(ToolchainLibraryFinder)
find_package(ALSA REQUIRED)

# Find PortAudio for eSpeak
find_package(PortAudio REQUIRED)

ToolchainLibraryFinder(
  NAME eSpeak
  HEADER espeak/speak_lib.h
  LIBRARY espeak
)

target_link_libraries(eSpeak::eSpeak INTERFACE ALSA::ALSA)
