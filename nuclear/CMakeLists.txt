# We use additional modules for the NUClear roles system
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

# We need NUClear
find_package(NUClear REQUIRED)
include_directories(SYSTEM ${NUClear_INCLUDE_DIR})

# Set to off to ignore building tests
option(BUILD_TESTS "Builds all of the tests for each module." OFF)

# Set to on to build as shared libraries
option(NUCLEAR_SHARED_BUILD "Build each module as a separate shared library." ON)

# Our banner file for placing at the top of the roles
set(NUCLEAR_ROLE_BANNER_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/roles/banner.png"
    CACHE PATH "The path the banner to print at the start of each role execution"
)

# Our location of our nuclear roles directory
set(NUCLEAR_ROLES_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}"
    CACHE PATH "The path to the nuclear roles system directory"
)

set(NUCLEAR_ADDITIONAL_SHARED_LIBRARIES
    ""
    CACHE STRING "Additional libraries used when linking roles, extensions, and utilities"
)

set(NUCLEAR_TEST_LIBRARIES
    ""
    CACHE STRING "Additional libraries used when linking module tests"
)

# Our variables that are used to locate the shared, module, and message folders They are given relative to the current
# project directory
set(NUCLEAR_MODULE_DIR
    "module"
    CACHE PATH "The path to the module directory for NUClear"
)
set(NUCLEAR_SHARED_DIR
    "shared"
    CACHE PATH "The path to the module directory for NUClear"
)
set(NUCLEAR_MESSAGE_DIR
    "${NUCLEAR_SHARED_DIR}/message"
    CACHE PATH "The path to the message directory for NUClear"
)
set(NUCLEAR_UTILITY_DIR
    "${NUCLEAR_SHARED_DIR}/utility"
    CACHE PATH "The path to the utility dir for NUClear"
)
set(NUCLEAR_EXTENSION_DIR
    "${NUCLEAR_SHARED_DIR}/extension"
    CACHE PATH "The path to the extension dir for NUClear"
)

# You generally shouldn't have to change these
mark_as_advanced(
  NUCLEAR_ROLE_BANNER_FILE NUCLEAR_MODULE_DIR NUCLEAR_MESSAGE_DIR NUCLEAR_UTILITY_DIR NUCLEAR_EXTENSION_DIR
)

# Make our shared directory to output files too
file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/${NUCLEAR_SHARED_DIR})

# Settings for the compiler to make NUClear work
include(NUClearCompilerSettings)

# Add the subdirectory for our messages
add_subdirectory("message")

# Add the subdirectory for our utilities Is after messages as it can use messages
add_subdirectory("utility")

# Add the subdirectory for our extensions
add_subdirectory("extension")

# Add the subdirectory for our roles
add_subdirectory("roles")

# Add the subdirectory for module This must be after roles as roles determines which modules to build
add_subdirectory("module")
