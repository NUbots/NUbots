# Build our NUClear module

find_package(OpenVINO REQUIRED)
find_package(TBB REQUIRED)

# Build our NUClear module
nuclear_module(LIBRARIES TBB::tbb openvino::runtime ${OpenVINO_LIBRARIES})
