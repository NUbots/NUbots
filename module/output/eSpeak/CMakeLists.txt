# Find the eSpeak Library
FIND_PACKAGE(eSpeak REQUIRED)

# Build our NUClear module
NUCLEAR_MODULE(INCLUDES ${eSpeak_INCLUDE_DIRS}
               LIBRARIES ${eSpeak_LIBRARIES})
