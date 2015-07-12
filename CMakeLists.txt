CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(NUbots)

SET(VERSION_MAJOR 0)
SET(VERSION_MINOR 2)
SET(VERSION_PATCH 0)

# Set to off to ignore building tests
OPTION(BUILD_TESTS "Builds all of the tests for each module." OFF)

# Set to on to build the report
OPTION(BUILD_REPORT "Builds the report from the latex source files." OFF)

# Set to on to build as shared libraries
OPTION(SHARED_BUILD "Build the binaries by linking to shared libraries." ON)

# Set on to prefer .a files over .so files for linking
OPTION(STATIC_LIBRARIES "Prefer using static .a files rather than .so files. NOTE: This must be enabled from the start!" OFF)

IF(STATIC_LIBRARIES)
    SET(CMAKE_FIND_LIBRARY_SUFFIXES
        ${CMAKE_STATIC_LIBRARY_SUFFIX}
        ${CMAKE_SHARED_LIBRARY_SUFFIX})
ENDIF()

# We use additional modules that cmake needs to know about
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/tools/cmake/")

# Setup our compiler settings
INCLUDE(Compiler)

# Setup our shared includes
INCLUDE(SharedLibraries)

# Include our module builder
INCLUDE(NUClearModuleBuilder)

# Include our shared messages etc headers
INCLUDE_DIRECTORIES("shared")
INCLUDE_DIRECTORIES("${CMAKE_BINARY_DIR}/shared")

# This will generate any .proto files in the messages folder
ADD_SUBDIRECTORY(shared/messages)
# This will build our Utilities library (common functionality)
ADD_SUBDIRECTORY(shared/utility)

# Add these to our shared libraries (started in our shared libraries cmake)
# We include utility again at the end for any "shim" functions that are needed in libraries
SET(NUBOTS_SHARED_LIBRARIES messages utility ${NUBOTS_SHARED_LIBRARIES} utility)

# Build tests in test mode
IF(BUILD_TESTS)
    ENABLE_TESTING()

    # Add the tests subdirectories
    ADD_SUBDIRECTORY("tests/shared/utility")
ENDIF()

# Clear or NUClear Modules cache variable
UNSET(NUCLEAR_MODULES CACHE)

# Find and build all our roles
ADD_SUBDIRECTORY(roles)

# Remove any duplicate modules (used in more then one role)
SET(modules ${NUCLEAR_MODULES})
LIST(REMOVE_DUPLICATES modules)
SET(NUCLEAR_MODULES ${modules} CACHE INTERNAL "A list of the modules in use by the system" FORCE)

# Clear our report files before reading (cmake scopes suck... could be fixed one day but who cares!)
UNSET(report_tex_files CACHE)
UNSET(report_bib_files CACHE)

# Build each of our modules and their corresponding tests
FOREACH(module ${NUCLEAR_MODULES})
    STRING(REPLACE "::" "/" modulepath "${module}")
    STRING(REPLACE "::" "" modulename "${module}")

    # Add the subdirectories
    ADD_SUBDIRECTORY(modules/${modulepath})
ENDFOREACH()

IF(BUILD_REPORT)
    ADD_SUBDIRECTORY(report)
ENDIF(BUILD_REPORT)

# Include our scp darwin sender
INCLUDE(DarwinSend)
