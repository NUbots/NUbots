# Find the eSpeak Library
find_package(eSpeak REQUIRED)

# Build our NUClear module
nuclear_module(INCLUDES ${eSpeak_INCLUDE_DIRS} LIBRARIES ${eSpeak_LIBRARIES})
