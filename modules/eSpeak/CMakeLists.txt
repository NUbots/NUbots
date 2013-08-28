# Set our module name so that the rest of this file can be generated
SET(module_name "eSpeak")

# YOU SHOULD NOT HAVE TO TOUCH ANYTHING BELOW THIS LINE
INCLUDE(${CMAKE_SOURCE_DIR}/cmake/Includes/BuildModule.cmake)

FIND_PACKAGE(eSpeak REQUIRED)
INCLUDE_DIRECTORIES(${ESPEAK_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(${module_name} ${ESPEAK_LIBRARIES})
TARGET_LINK_LIBRARIES(${module_name}Tests ${ESPEAK_LIBRARIES})
