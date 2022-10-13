# Build our NUClear module
find_package(vosk-api REQUIRED)
nuclear_module(LIBRARIES vosk-api)
