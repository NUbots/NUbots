SET(NUBOTS_PLATFORM "DarwinOP" CACHE STRING "The platform that the code will be optimised for")

# Tune for the Darwin-OP (uses a http://ark.intel.com/products/35463)
# -mmovbe isn't enabled as it was specific to atom CPUs until haswell
# and won't run on computers before that (the development machines)
IF(NUBOTS_PLATFORM STREQUAL "DarwinOP")
    SET(NUBOTS_PLATFORM_FLAGS "-march=bonnell -mtune=bonnell -m32 -mfxsr -mno-movbe -mmmx -msahf -msse -msse2 -msse3 -mssse3 --param l1-cache-size=24 --param l1-cache-line-size=64 --param l2-cache-size=512"
        CACHE INTERNAL "The compiler flags to use for the selected platform" FORCE)
ENDIF()

# Add the flags to our list
SET(CMAKE_CXX_FLAGS                "${CMAKE_CXX_FLAGS} ${NUBOTS_PLATFORM_FLAGS}")
SET(CMAKE_C_FLAGS                  "${CMAKE_C_FLAGS} ${NUBOTS_PLATFORM_FLAGS}")
SET(CMAKE_CXX_FLAGS_DEBUG          "${CMAKE_CXX_FLAGS_DEBUG} ${NUBOTS_PLATFORM_FLAGS}")
SET(CMAKE_C_FLAGS_DEBUG            "${CMAKE_C_FLAGS_DEBUG} ${NUBOTS_PLATFORM_FLAGS}")
SET(CMAKE_CXX_FLAGS_RELEASE        "${CMAKE_CXX_FLAGS_RELEASE} ${NUBOTS_PLATFORM_FLAGS}")
SET(CMAKE_C_FLAGS_RELEASE          "${CMAKE_C_FLAGS_RELEASE} ${NUBOTS_PLATFORM_FLAGS}")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${NUBOTS_PLATFORM_FLAGS}")
SET(CMAKE_C_FLAGS_RELWITHDEBINFO   "${CMAKE_C_FLAGS_RELWITHDEBINFO} ${NUBOTS_PLATFORM_FLAGS}")
SET(CMAKE_CXX_FLAGS_MINSIZEREL     "${CMAKE_CXX_FLAGS_MINSIZEREL} ${NUBOTS_PLATFORM_FLAGS}")
SET(CMAKE_C_FLAGS_MINSIZEREL       "${CMAKE_C_FLAGS_MINSIZEREL} ${NUBOTS_PLATFORM_FLAGS}")