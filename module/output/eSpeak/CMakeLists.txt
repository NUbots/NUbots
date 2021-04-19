# Find the eSpeak Library
find_package(eSpeak REQUIRED)

# Build our NUClear module
nuclear_module(LIBRARIES eSpeak::eSpeak)
