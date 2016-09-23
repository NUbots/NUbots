SET(NUBOTS_PLATFORM "DarwinOP" CACHE STRING "The platform that the code will be optimised for")

# Tune for the Darwin-OP (uses a http://ark.intel.com/products/35463)
# -mmovbe isn't enabled as it was specific to atom CPUs until haswell
# and won't run on computers before that (the development machines)
IF(NUBOTS_PLATFORM STREQUAL "DarwinOP")
    ADD_COMPILE_OPTIONS(-march=bonnell
                        -mtune=bonnell
                        -m32
                        -mfxsr
                        -mno-movbe
                        -mmmx
                        -msahf
                        -msse
                        -msse2
                        -msse3
                        -mssse3)
    # These options don't seem to work in ADD_COMPILE_OPTIONS because they are two args
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --param l1-cache-size=24 --param l1-cache-line-size=64 --param l2-cache-size=512")
ENDIF()
