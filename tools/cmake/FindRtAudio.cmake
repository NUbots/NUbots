INCLUDE(ToolchainLibraryFinder)
ToolchainLibraryFinder(NAME RtAudio
                       HEADER RtAudio.h
                       LIBRARY rtaudio
                       VERSION_FILE RtAudio.h
                       VERSION_REGEX #define RTAUDIO_VERSION "4.1.1"
)