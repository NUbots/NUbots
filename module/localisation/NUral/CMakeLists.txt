# Build our NUClear module
set(OpenVINO_DIR /usr/local/lib/cmake)
find_package(OpenVINO REQUIRED)

nuclear_module(LIBRARIES openvino::runtime ${OpenVINO_LIBRARIES})
