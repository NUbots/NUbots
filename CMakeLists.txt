CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(Robocup)

SET(VERSION_MAJOR 0)
SET(VERSION_MINOR 2)
SET(VERSION_PATCH 0)

# These define which modules will be installed
SET(modules
    ConfigSystem
    DarwinPlatform
    DarwinCameraReader
    ScriptEngine
    eSpeak
    PartyDarwin
    AudioInput
    NUBugger)

# Set to off to ignore building tests
OPTION(BUILD_TESTS "Builds all of the tests for each module." ON)

# We use additional modules that cmake needs to know about
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

# Setup our compiler settings
INCLUDE("${CMAKE_SOURCE_DIR}/cmake/Includes/Compiler.cmake")

# Setup our shared includes
INCLUDE("${CMAKE_SOURCE_DIR}/cmake/Includes/SharedLibraries.cmake")

# Include our module builder
INCLUDE("${CMAKE_SOURCE_DIR}/cmake/Includes/NUClearModuleBuilder.cmake")

# Include our scp darwin sender
INCLUDE("${CMAKE_SOURCE_DIR}/cmake/Includes/DarwinSend.cmake")

# Include our shared messages etc headers
INCLUDE_DIRECTORIES("shared")
INCLUDE_DIRECTORIES("build/shared")

IF(BUILD_TESTS)
    ENABLE_TESTING()
ENDIF()

# This will generate any .proto files in the messages folder
ADD_SUBDIRECTORY(shared/messages)
# This will build our Utilities library (common functionality)
ADD_SUBDIRECTORY(shared/utility)

# Add these to our shared libraries (started in our shared libraries cmake)
SET(NUBOTS_SHARED_LIBRARIES ${NUBOTS_SHARED_LIBRARIES} messages utility)

# Build each of our modules and their corresponding tests
FOREACH(module ${modules})
    # Add the subdirectories
    ADD_SUBDIRECTORY(modules/${module})
    INCLUDE_DIRECTORIES(modules/${module}/src)
    ADD_TEST(${module}Tests modules/${module}/${module}Tests)
ENDFOREACH()

ADD_EXECUTABLE(robocup main.cpp)
TARGET_LINK_LIBRARIES(robocup
    ${NUBOTS_SHARED_LIBRARIES}
    ${modules})
