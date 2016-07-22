cmake_minimum_required(VERSION 3.4)

# This directory contains our message types and other shared types
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/shared/)

IF(OPENGL_FOUND)
	FILE(GLOB_RECURSE opengl "opengl/**.cpp" "opengl/**.c" "opengl/**.h")
	SET(src ${src} opengl)
ENDIF()

# This gets all of our source files
FILE(GLOB_RECURSE src "*/**.cpp" "*/**.c" "*/**.h")

# Build a library from these files
ADD_LIBRARY(utility ${src})

# Put it in an IDE group for shared
SET_PROPERTY(TARGET utility PROPERTY FOLDER "shared/")
