# Common C++ Flags
SET(CMAKE_CXX_FLAGS "-std=c++11 -O3 -DNDEBUG -march=atom -mtune=atom")
SET(CMAKE_C_FLAGS "-O3 -DNDEBUG")

# XCode support
IF("${CMAKE_GENERATOR}" MATCHES "Xcode")
    message("Enabling xcode support")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++0x")
ENDIF()

IF("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU")
    SET(GCC_MINIMUM_VERSION 4.7)
    EXECUTE_PROCESS(
        COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    IF(GCC_VERSION VERSION_LESS ${GCC_MINIMUM_VERSION})
        message(FATAL_ERROR "${PROJECT_NAME} requires g++ 4.7.2 or greater.")
    ENDIF()

    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -Wall")

ELSEIF("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wpedantic -stdlib=libc++")

ELSE()
    MESSAGE(FATAL_ERROR "Unsupported compiler!")
ENDIF()
