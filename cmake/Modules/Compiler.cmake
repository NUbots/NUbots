# We need noncall exceptions so we can throw exceptions from signal handlers
# This allows us to catch null pointer exceptions
ADD_COMPILE_OPTIONS(-Wall
                    -Wpedantic
                    -Wextra
                    -Weffc++
                    -Werror
                    -fnon-call-exceptions)

SET(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_RPATH} toolchain/)

# GNU Compiler
IF(CMAKE_CXX_COMPILER_ID MATCHES GNU)
    # Enable colours on g++ 4.9 or greater
    IF(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9 OR CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 4.9)
        ADD_COMPILE_OPTIONS(-fdiagnostics-color=always)
    ENDIF()
ENDIF()

SET(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_RPATH} toolchain /home/darwin/toolchain)

# Disable armadillo bounds checking in release and min size release builds
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DARMA_NO_DEBUG")
SET(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -DARMA_NO_DEBUG")
