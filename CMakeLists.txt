# This directory contains our message types and other shared types
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/shared/ ${CMAKE_SOURCE_DIR}/)

#Get files to compile
INCLUDE(GetNUtilities.cmake)

# Build a library from these files
ADD_LIBRARY(utility ${NUTILITIES_SRC_FILES})

# Put it in an IDE group for shared
SET_PROPERTY(TARGET utility PROPERTY FOLDER "shared/")
