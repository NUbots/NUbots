# We need noncall exceptions so we can throw exceptions from signal handlers
# This allows us to catch null pointer exceptions
ADD_COMPILE_OPTIONS(-Wall
                    -Wpedantic
                    -Wextra
                    -fnon-call-exceptions)

LIST(APPEND CMAKE_INSTALL_RPATH toolchain/)
LIST(APPEND CMAKE_INSTALL_RPATH /home/nubots/toolchain/)
LIST(APPEND CMAKE_INSTALL_RPATH /nubots/toolchain/${PLATFORM}/lib/)
LIST(APPEND CMAKE_INSTALL_RPATH /nubots/toolchain/lib/)

# GNU Compiler
IF(CMAKE_CXX_COMPILER_ID MATCHES GNU)
    # Enable colours on g++ 4.9 or greater
    IF(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9 OR CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 4.9)
        ADD_COMPILE_OPTIONS(-fdiagnostics-color=always)
    ENDIF()
ENDIF()

SET(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_RPATH} /home/nubots/toolchain/)
