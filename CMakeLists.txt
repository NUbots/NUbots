CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(Robocup)

SET(VERSION_MAJOR 0)
SET(VERSION_MINOR 2)
SET(VERSION_PATCH 0)

OPTION(BUILD_TESTS "Builds all of the tests for each module." ON)

# We use additional modules that cmake needs to know about
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

# Common C++ Flags
SET(CMAKE_CXX_FLAGS "-std=c++11")
SET(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

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

FIND_PACKAGE(NUClear REQUIRED)
FIND_PACKAGE(ZMQ REQUIRED)
FIND_PACKAGE(Protobuf REQUIRED)
INCLUDE_DIRECTORIES(${NUCLEAR_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${ZMQ_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIRS})

IF(BUILD_TESTS)
    ENABLE_TESTING()
ENDIF()

# This will generate any .proto files in the messages folder
ADD_SUBDIRECTORY(shared/messages)

SET(modules
    DarwinPlatform
    NUBugger)

FOREACH(module ${modules})
    # Add the subdirectories
    ADD_SUBDIRECTORY(modules/${module})
    INCLUDE_DIRECTORIES(modules/${module}/src)
    ADD_TEST(${module}Tests modules/${module}/${module}Tests)
ENDFOREACH()

ADD_EXECUTABLE(robocup main.cpp)
TARGET_LINK_LIBRARIES(robocup
    messages
    ${modules}
    ${NUCLEAR_LIBRARIES}
    ${ZMQ_LIBRARIES}
    ${PROTOBUF_LIBRARIES})