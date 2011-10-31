##############################
# wenao.cmake
# 
#   - set TARGET_ROBOT_DIR to Darwin
#   - set CMAKE_MODULES_PATH to ./CMakeModules
#   - set NUBOT_IS_EXECUTABLE
#   - set the OUTPUT_ROOT_DIR_EXE to 
#   - print debug information if desired

INCLUDE(${TARGET_ROBOT_DIR}/cmake/sources.cmake)

############################ CMAKE PACKAGE DIRECTORY
# Set cmakeModules folder
SET( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules )

######### Set NUBOT_EXECUTABLE so that the code is compiled into an executable
SET(NUBOT_IS_EXECUTABLE ON)

######### Try and find where OPENCV is installed (for now Darwin uses opencv for image capture)
FIND_PACKAGE(OPENCV REQUIRED)
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS})
LIST(APPEND NUBOT_LINK_LIBRARIES ${OpenCV_LIBS})

FIND_PACKAGE(DARWIN REQUIRED)
INCLUDE_DIRECTORIES( ${DARWIN_INCLUDE_DIR})
LIST(APPEND NUBOT_LINK_LIBRARIES ${DARWIN_LIBRARIES})


SET( OUTPUT_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Build/Darwin/" )

