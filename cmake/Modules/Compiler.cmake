# We need noncall exceptions so we can throw exceptions from signal handlers. This allows us to catch null pointer
# exceptions. no-int-in-bool-context is to silence a warning from Eigen.
add_compile_options(-Wall -Wpedantic -Wextra -Wno-int-in-bool-context -fnon-call-exceptions)

# C++17 allows eigen to not need to worry about alignment
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Make sure our binaries can find our libraries both in docker and on the robot
list(APPEND CMAKE_INSTALL_RPATH /home/nubots/toolchain/)
list(APPEND CMAKE_INSTALL_RPATH /usr/local/lib)

# GNU Compiler
if(CMAKE_CXX_COMPILER_ID MATCHES GNU)
  # Enable colours on g++ 4.9 or greater
  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9 OR CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 4.9)
    add_compile_options(-fdiagnostics-color=always)
  endif()
endif()

# Disable armadillo bounds checking in release and min size release builds
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DARMA_NO_DEBUG")
set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -DARMA_NO_DEBUG")
