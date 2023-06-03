# Build our NUClear module
find_package(vosk REQUIRED)
nuclear_module(LIBRARIES vosk::vosk)
