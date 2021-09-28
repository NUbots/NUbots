find_package(OpenCV 3.4.15 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

# Build our NUClear module
nuclear_module(LIBRARIES ${OpenCV_LIBRARIES} ${Pangolin_LIBRARIES})
