# Common C++ Flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -Wpedantic -Wextra -g -march=atom -mtune=atom -fnon-call-exceptions")
SET(CMAKE_C_FLAGS "-O3 -DNDEBUG")

# XCode support
IF("${CMAKE_GENERATOR}" MATCHES "Xcode")
    message("Enabling xcode support")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++0x")
ENDIF()

IF("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU")
    SET(GCC_MINIMUM_VERSION 4.8)

    EXECUTE_PROCESS(
        COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)

    IF(GCC_VERSION VERSION_LESS ${GCC_MINIMUM_VERSION})
        message(FATAL_ERROR "${PROJECT_NAME} requires g++ 4.7.2 or greater.")
    ENDIF()

    IF(GCC_VERSION VERSION_GREATER "4.9" OR GCC_VERSION VERSION_EQUAL "4.9")
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always")
    ENDIF()

    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

ELSEIF("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wpedantic -stdlib=libc++")

ELSE()
    MESSAGE(FATAL_ERROR "Unsupported compiler!")
ENDIF()

