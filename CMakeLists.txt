# This CMake file is for building the utilities without external code dependencies.

cmake_minimum_required(VERSION 3.4)

IF(NOT MSVC)
	# Compilation must be done with c++14 for NUClear to work
    ADD_COMPILE_OPTIONS(-std=c++14 -fPIC)
ENDIF()

FIND_PACKAGE(ARMADILLO REQUIRED
            NAMES Armadillo)

INCLUDE_DIRECTORIES(
	${CMAKE_SOURCE_DIR}
	${ARMADILLO_INCLUDE_DIR})

add_subdirectory(utility)
