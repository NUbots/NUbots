CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(NUbots)

SET(VERSION_MAJOR 0)
SET(VERSION_MINOR 2)
SET(VERSION_PATCH 0)

# Set to off to ignore building tests
OPTION(BUILD_TESTS "Builds all of the tests for each module." OFF)

# Set to on to build as shared libraries
OPTION(SHARED_BUILD "Build the binaries by linking to shared libraries." ON)

# We use additional modules that cmake needs to know about
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

# Setup our compiler settings
INCLUDE("${CMAKE_SOURCE_DIR}/cmake/Includes/Compiler.cmake")

# Setup our shared includes
INCLUDE("${CMAKE_SOURCE_DIR}/cmake/Includes/SharedLibraries.cmake")

# Include our module builder
INCLUDE("${CMAKE_SOURCE_DIR}/cmake/Includes/NUClearModuleBuilder.cmake")

# Include our shared messages etc headers
INCLUDE_DIRECTORIES("shared")
INCLUDE_DIRECTORIES("${CMAKE_BINARY_DIR}/shared")

# This will generate any .proto files in the messages folder
ADD_SUBDIRECTORY(shared/messages)
# This will build our Utilities library (common functionality)
ADD_SUBDIRECTORY(shared/utility)

# Add these to our shared libraries (started in our shared libraries cmake)
SET(NUBOTS_SHARED_LIBRARIES ${NUBOTS_SHARED_LIBRARIES} messages utility)

# Build tests in test mode
IF(BUILD_TESTS)
    ENABLE_TESTING()

    # Add the tests subdirectories
    ADD_SUBDIRECTORY("tests/shared/utility")
ENDIF()

# Clear or NUClear Modules cache variable
SET(NUCLEAR_MODULES ${NUCLEAR_MODULES} ${role_modules} CACHE INTERNAL "A list of the modules in use by the system" FORCE)

# Find and build all our roles
ADD_SUBDIRECTORY(roles)

# Remove any duplicate modules (used in more then one role)
SET(modules ${NUCLEAR_MODULES})
LIST(REMOVE_DUPLICATES modules)
SET(NUCLEAR_MODULES ${modules} CACHE INTERNAL "A list of the modules in use by the system" FORCE)

# Clear our report files before reading (cmake scopes suck... could be fixed one day but who cares!)
SET(report_tex_files "" CACHE INTERNAL "List of report tex files" FORCE)
SET(report_bib_files "" CACHE INTERNAL "List of report bib files" FORCE)

# Build each of our modules and their corresponding tests
FOREACH(module ${NUCLEAR_MODULES})
    STRING(REPLACE "::" "/" modulepath "${module}")
    STRING(REPLACE "::" "" modulename "${module}")

    # Add the subdirectories
    ADD_SUBDIRECTORY(modules/${modulepath})
ENDFOREACH()

ADD_SUBDIRECTORY(report)

# Include our scp darwin sender
INCLUDE("${CMAKE_SOURCE_DIR}/cmake/Includes/DarwinSend.cmake")
