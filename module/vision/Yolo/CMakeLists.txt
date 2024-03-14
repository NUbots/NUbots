find_package(OpenCV REQUIRED)
set(OpenVINO_DIR /usr/local/runtime/cmake)
set(InferenceEngine_DIR /usr/local/runtime/cmake)
find_package(OpenVINO REQUIRED)
find_package(InferenceEngine REQUIRED)
find_package(TBB REQUIRED)


# Build our NUClear module
nuclear_module(LIBRARIES ${OpenCV_LIBRARIES} TBB::tbb openvino::runtime ${OpenVINO_LIBRARIES} ${InferenceEngine_LIBRARIES} )
#include_directories(/usr/local/runtime/include)
#link_directories(/usr/local/runtime/lib/intel64/)
