# Set our module name so that the rest of this file can be generated
SET(module_name "DarwinMotors")

# YOU SHOULD NOT HAVE TO TOUCH ANYTHING BELOW THIS LINE

# Find all our files
FILE(GLOB src "src/**.cpp", "src/**.h")

# Include our own src dir and the shared dir
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/shared)

# Add all our code to a library
ADD_LIBRARY(${module_name} ${src})