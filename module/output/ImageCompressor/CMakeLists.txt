find_package(TurboJPEG REQUIRED)
find_package(va REQUIRED)
find_package(OpenCL REQUIRED)

add_compile_definitions(CL_TARGET_OPENCL_VERSION=120)

# Build our NUClear module
nuclear_module(LIBRARIES TurboJPEG::TurboJPEG OpenCL::OpenCL va::va)
