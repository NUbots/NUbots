# Build our NUClear module
find_package(OpenCV REQUIRED)

nuclear_module(LIBRARIES
    ${OpenCV_LIBRARIES}
)
