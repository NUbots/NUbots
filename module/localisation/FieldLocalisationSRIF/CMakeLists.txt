find_package(OpenCV REQUIRED)

# Build our NUClear module
nuclear_module(LIBRARIES ${OpenCV_LIBRARIES})
