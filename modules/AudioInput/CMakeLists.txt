# Set our module name so that the rest of this file can be generated
SET(module_name "AudioInput")

# YOU SHOULD NOT HAVE TO TOUCH ANYTHING BELOW THIS LINE
INCLUDE(${CMAKE_SOURCE_DIR}/cmake/Includes/BuildModule.cmake)

FIND_PACKAGE(RtAudio REQUIRED)
INCLUDE_DIRECTORIES(${RTAUDIO_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(${module_name} ${RTAUDIO_LIBRARY})
TARGET_LINK_LIBRARIES(${module_name}Tests ${RTAUDIO_LIBRARY})
