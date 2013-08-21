# Set our module name so that the rest of this file can be generated
SET(module_name "AudioInput")

# YOU SHOULD NOT HAVE TO TOUCH ANYTHING BELOW THIS LINE
INCLUDE(${CMAKE_SOURCE_DIR}/cmake/Includes/BuildModule.cmake)

FIND_PACKAGE(OpenAL REQUIRED)
INCLUDE_DIRECTORIES(${OPENAL_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(${module_name} ${OPENAL_LIBRARY})