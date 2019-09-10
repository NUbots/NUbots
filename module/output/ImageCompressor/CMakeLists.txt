# Build our NUClear module
find_package(TurboJPEG REQUIRED)

# Build our NUClear module
nuclear_module(INCLUDES ${TurboJPEG_INCLUDE_DIRS} LIBRARIES ${TurboJPEG_LIBRARIES})
