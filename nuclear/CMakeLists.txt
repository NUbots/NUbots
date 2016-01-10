# We need NUClear
FIND_PACKAGE(NUClear REQUIRED)

# Set to off to ignore building tests
OPTION(BUILD_TESTS "Builds all of the tests for each module." OFF)

# Set to on to build as shared libraries
OPTION(NUCLEAR_SHARED_BUILD "Build each module as a separate shared library." ON)

# Our variables that are used to locate the shared, module, and message folders
SET(NUCLEAR_MODULE_DIR "${CMAKE_SOURCE_DIR}/module"              CACHE PATH "The path to the module directory for NUClear")
SET(NUCLEAR_MESSAGE_DIR "${CMAKE_SOURCE_DIR}/shared/message"     CACHE PATH "The path to the message directory for NUClear")
SET(NUCLEAR_UTILITY_DIR "${CMAKE_SOURCE_DIR}/shared/utility"     CACHE PATH "The path to the utility dir for NUClear")
SET(NUCLEAR_EXTENSION_DIR "${CMAKE_SOURCE_DIR}/shared/extension" CACHE PATH "The path to the extension dir for NUClear")

# You generally shouldn't have to change these
MARK_AS_ADVANCED(NUCLEAR_MODULE_DIR NUCLEAR_MESSAGE_DIR NUCLEAR_UTILITY_DIR NUCLEAR_EXTENSION_DIR)

# We use additional modules for the NUClear roles system
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

# Settings for the compiler to make NUClear work
INCLUDE(NUClearCompilerSettings)

# Add the subdirectory for our utilities
ADD_SUBDIRECTORY("utility")

# Add the subdirectory for our extensions
ADD_SUBDIRECTORY("extension")

# Add the subdirectory for our messages
ADD_SUBDIRECTORY("message")

# Add the subdirectory for our roles
ADD_SUBDIRECTORY("roles")

# Add the subdirectory for module
# This must be after roles as roles determines which modules to build
ADD_SUBDIRECTORY("module")