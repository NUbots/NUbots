# We use additional modules for the NUClear roles system
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

# Set to off to ignore building tests
option(BUILD_TESTS "Builds all of the tests for each module." OFF)

# The ways we can link the libraries together
if(NOT NUCLEAR_LINK_TYPE)
  set(NUCLEAR_LINK_TYPE
      SHARED
      CACHE STRING "Choose method to link the binary" FORCE
  )
  # Set the possible values of build type for cmake-gui
  set_property(CACHE NUCLEAR_LINK_TYPE PROPERTY STRINGS "SHARED" "STATIC" "OBJECT")
endif()

# RPath variables use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH FALSE)

# Build the RPATH into the binary before install
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# Make OSX use the same RPATH as everyone else
set(CMAKE_MACOSX_RPATH ON)

# Add some useful places to the RPATH. These will allow the binary to run from the build folder
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_RPATH} lib/ ../lib/ bin/lib)

# Our banner file for placing at the top of the roles
set(NUCLEAR_ROLE_BANNER_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/roles/banner.png"
    CACHE PATH "The path the banner to print at the start of each role execution"
)

# We need -fPIC for all code
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Our variables that are used to locate the shared, module, and message folders They are given relative to the current
# project directory
set(NUCLEAR_MODULE_DIR
    "module"
    CACHE PATH "The path to the module directory for NUClear"
)
set(NUCLEAR_ROLES_DIR
    "roles"
    CACHE PATH "The path to the nuclear roles system directory"
)
set(NUCLEAR_SHARED_DIR
    "shared"
    CACHE PATH "The path to the shared directory for NUClear"
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
  NUCLEAR_MODULE_DIR
  NUCLEAR_ROLES_DIR
  NUCLEAR_SHARED_DIR
  NUCLEAR_MESSAGE_DIR
  NUCLEAR_UTILITY_DIR
  NUCLEAR_EXTENSION_DIR
  NUCLEAR_ROLE_BANNER_FILE
)

# Make our shared directory to output files too
file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/${NUCLEAR_SHARED_DIR})

# Add the subdirectory for our messages
add_subdirectory("message")

# Add the subdirectory for our utilities. Is after messages as it can use messages
add_subdirectory("utility")

# Add the subdirectory for our extensions
add_subdirectory("extension")

# Add the subdirectory for our roles
add_subdirectory("roles")

# Add the subdirectory for module. This must be after roles as roles determines which modules to build
add_subdirectory("module")

if(BUILD_TESTS)
  add_subdirectory(tests)
endif(BUILD_TESTS)
