# Build our NUClear module

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

nuclear_module(LIBRARIES ${OpenCV_LIBRARIES} ${Pangolin_LIBRARIES})
