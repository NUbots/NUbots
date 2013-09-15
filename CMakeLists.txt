CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(Robocup)

SET(VERSION_MAJOR 0)
SET(VERSION_MINOR 2)
SET(VERSION_PATCH 0)

# Set to off to ignore building tests
OPTION(BUILD_TESTS "Builds all of the tests for each module." ON)

# We use additional modules that cmake needs to know about
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

# Our scripts directory contains shell scripts for running, generating, installing, etc
# not to be confused with robot scripts.
SET(NUBOTS_SCRIPTS_DIR "${CMAKE_SOURCE_DIR}/scripts/")

# Setup our compiler settings
INCLUDE("${CMAKE_SOURCE_DIR}/cmake/Includes/Compiler.cmake")

# Setup our shared includes
INCLUDE("${CMAKE_SOURCE_DIR}/cmake/Includes/SharedLibraries.cmake")

# Include our module builder
INCLUDE("${CMAKE_SOURCE_DIR}/cmake/Includes/NUClearModuleBuilder.cmake")

# Include our shared messages etc headers
INCLUDE_DIRECTORIES("shared")
INCLUDE_DIRECTORIES("build/shared")

# Build tests in test mode
IF(BUILD_TESTS)
    ENABLE_TESTING()
ENDIF()

# This will generate any .proto files in the messages folder
ADD_SUBDIRECTORY(shared/messages)
# This will build our Utilities library (common functionality)
ADD_SUBDIRECTORY(shared/utility)

# Add these to our shared libraries (started in our shared libraries cmake)
SET(NUBOTS_SHARED_LIBRARIES ${NUBOTS_SHARED_LIBRARIES} messages utility)

# Find and build all our hats
ADD_SUBDIRECTORY(hats)

# Remove any duplicate modules (used in more then one hat)
LIST(REMOVE_DUPLICATES modules)

# Build each of our modules and their corresponding tests
FOREACH(module ${modules})
    STRING(REPLACE "::" "/" modulepath "${module}")
    STRING(REPLACE "::" "" modulename "${module}")

    # Add the subdirectories
    ADD_SUBDIRECTORY(modules/${modulepath})
    # Add the test
    ADD_TEST(Test${modulename} modules/${modulepath}/Test${modulename})
ENDFOREACH()

# Include our scp darwin sender
INCLUDE("${CMAKE_SOURCE_DIR}/cmake/Includes/DarwinSend.cmake")
