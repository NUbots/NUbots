CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
# This directory contains our message types and other shared types
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/shared/ ${CMAKE_SOURCE_DIR}/ SYSTEM)
INCLUDE_DIRECTORIES(${NUCLEAR_MESSAGE_INCLUDE_DIRS})

# We need Eigen3
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(SYSTEM ${Eigen3_INCLUDE_DIRS})

# Required for vision utility
FIND_PACKAGE(Aravis REQUIRED)
FIND_PACKAGE(glib2 REQUIRED)
INCLUDE_DIRECTORIES(SYSTEM ${Aravis_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(SYSTEM ${glib2_INCLUDE_DIRS})

#Set project name found variable
STRING(TOUPPER ${CMAKE_PROJECT_NAME} PROJECT_NAME)
SET(${PROJECT_NAME}_PROJECT_FOUND TRUE)

#Include tools
INCLUDE(ConfigureUtilities.cmake)

#Note where we are
SET(NUTILITIES_DIR ${CMAKE_CURRENT_SOURCE_DIR})

#Find all subdirectories
SUBDIRLIST(directories ${CMAKE_CURRENT_SOURCE_DIR})

#initialise variables which will contain our collected files and libs
UNSET(NUTILITIES_LIBRARIES_FILES CACHE)

#Add each subdirectory with selection for loaded libraries
FOREACH(dir ${directories})
	ADD_SUBDIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/${dir})
ENDFOREACH()

#Compile the nutilities library
ADD_LIBRARY(nutilities ${NUTILITIES_LIBRARIES_FILES})

# Link in our messaging library to ensure they are built first
# Also link to our shared libraries
TARGET_LINK_LIBRARIES(nutilities ${NUCLEAR_MESSAGE_LIBRARIES})
TARGET_LINK_LIBRARIES(nutilities ${NUCLEAR_ADDITIONAL_SHARED_LIBRARIES})

# SET(NUCLEAR_UTILITY_LIBRARIES ${NUCLEAR_UTILITY_LIBRARIES} ${NUTILITIES_LIBRARIES} CACHE INTERNAL "Library for NUClear utilities" FORCE)
SET(NUCLEAR_UTILITY_LIBRARIES nutilities CACHE INTERNAL "Library for NUClear utilities" FORCE)
