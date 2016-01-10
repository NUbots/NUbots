# Default to do a debug build
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Debug CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
       FORCE)
ENDIF()

# RPath variables
# use, i.e. don't skip the full RPATH for the build tree
SET(CMAKE_SKIP_BUILD_RPATH FALSE)

# Build the RPATH into the binary before install
SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# Add some useful places to the RPATH
# These will allow the binary to run from the build folder
SET(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_RPATH} lib/ ../lib/ bin/lib)

# Compilation must be done with c++14 for NUClear to work
SET(NUCLEAR_ROLES_CXX_FLAGS "${NUCLEAR_ROLES_CXX_FLAGS} -std=c++14")
SET(NUCLEAR_ROLES_C_FLAGS   "${NUCLEAR_ROLES_C_FLAGS} -std=c++14")

# Build using -fPIC so the code can be shared libraries properly
SET(NUCLEAR_ROLES_CXX_FLAGS "${NUCLEAR_ROLES_CXX_FLAGS} -fPIC")
SET(NUCLEAR_ROLES_C_FLAGS   "${NUCLEAR_ROLES_C_FLAGS} -fPIC")

# Enable super strict warnings so developers behave themselves
SET(NUCLEAR_ROLES_CXX_FLAGS "${NUCLEAR_ROLES_CXX_FLAGS} -Wall -Wpedantic -Wextra")
SET(NUCLEAR_ROLES_C_FLAGS   "${NUCLEAR_ROLES_C_FLAGS} -Wall -Wpedantic -Wextra")

# Add these flags to the build types
SET(CMAKE_CXX_FLAGS                 "${CMAKE_CXX_FLAGS} ${NUCLEAR_ROLES_CXX_FLAGS}")
SET(CMAKE_C_FLAGS                   "${CMAKE_C_FLAGS} ${NUCLEAR_ROLES_C_FLAGS}")
SET(CMAKE_CXX_FLAGS_DEBUG           "${CMAKE_CXX_FLAGS_DEBUG} ${NUCLEAR_ROLES_CXX_FLAGS}")
SET(CMAKE_C_FLAGS_DEBUG             "${CMAKE_C_FLAGS_DEBUG} ${NUCLEAR_ROLES_C_FLAGS}")
SET(CMAKE_CXX_FLAGS_RELEASE         "${CMAKE_CXX_FLAGS_RELEASE} ${NUCLEAR_ROLES_CXX_FLAGS}")
SET(CMAKE_C_FLAGS_RELEASE           "${CMAKE_C_FLAGS_RELEASE} ${NUCLEAR_ROLES_C_FLAGS}")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO  "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${NUCLEAR_ROLES_CXX_FLAGS}")
SET(CMAKE_C_FLAGS_RELWITHDEBINFO    "${CMAKE_C_FLAGS_RELWITHDEBINFO} ${NUCLEAR_ROLES_C_FLAGS}")
