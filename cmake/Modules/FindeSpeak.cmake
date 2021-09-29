include(ToolchainLibraryFinder)

# Find ALSA for eSpeak
find_package(ALSA REQUIRED)

ToolchainLibraryFinder(
  NAME eSpeak
  HEADER espeak-ng/speak_lib.h
  LIBRARY espeak-ng
)

target_link_libraries(eSpeak::eSpeak INTERFACE ALSA::ALSA)
