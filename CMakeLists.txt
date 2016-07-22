# cmake_minimum_required(VERSION 3.4)

# This directory contains our message types and other shared types
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/shared/ ${CMAKE_SOURCE_DIR}/)
MESSAGE("Including NUbots utilities...")
MESSAGE(asdasdasdas)

# Get source files based on configuration of libraries and project
IF(${PROJECT_NAME} MATCHES NUbots)
	MESSAGE("Including NUbots utilities...")
	FILE(GLOB_RECURSE src "*/**.cpp" "*/**.c" "*/**.h")
ENDIF()

# Build a library from these files
ADD_LIBRARY(utility ${src})

# Put it in an IDE group for shared
SET_PROPERTY(TARGET utility PROPERTY FOLDER "shared/")
