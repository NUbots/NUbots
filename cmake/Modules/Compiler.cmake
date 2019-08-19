# We need noncall exceptions so we can throw exceptions from signal handlers This allows us to catch null pointer
# exceptions
add_compile_options(
  -Wall -Wpedantic -Wextra
  # -Werror
  # Because Eigen
  -Wno-int-in-bool-context -fnon-call-exceptions
)

list(APPEND CMAKE_INSTALL_RPATH toolchain/)
list(APPEND CMAKE_INSTALL_RPATH /home/darwin/toolchain/)
list(APPEND CMAKE_INSTALL_RPATH /home/nubots/toolchain/)
list(APPEND CMAKE_INSTALL_RPATH /nubots/toolchain/${PLATFORM}/lib/)
list(APPEND CMAKE_INSTALL_RPATH /nubots/toolchain/lib/)

# GNU Compiler
if(CMAKE_CXX_COMPILER_ID MATCHES GNU)
  # Enable colours on g++ 4.9 or greater
  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9 OR CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 4.9)
    add_compile_options(-fdiagnostics-color=always)
  endif()
endif()

set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_RPATH} /home/darwin/toolchain/ /home/nubots/toolchain/)

# Disable armadillo bounds checking in release and min size release builds
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DARMA_NO_DEBUG")
set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -DARMA_NO_DEBUG")
