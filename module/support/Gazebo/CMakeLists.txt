# Prevent FindFreeImage from trying to run in "cross-compiling" mode.
#SET(FREEIMAGE_RUNS 1)
#SET(FREEIMAGE_COMPILES 1)
#SET(FREEIMAGE_RUNS__TRYRUN_OUTPUT "")
# Find gazebo library
FIND_PACKAGE(ignition-transport4 REQUIRED)
#FIND_PACKAGE(ignition-msgs1 REQUIRED)
#ADD_COMPILE_OPTIONS(${GAZEBO_CXX_FLAGS})
# Build our NUClear module
#NUCLEAR_MODULE()
NUCLEAR_MODULE(#INCLUDES ${GAZEBO_INCLUDE_DIRS}
               LIBRARIES ${IGNITION-TRANSPORT_LIBRARIES})# ${ignition-msgs1_LIBRARIES})
# cmake -DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/native.cmake
