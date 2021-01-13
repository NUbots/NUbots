find_package(TurboJPEG REQUIRED)
find_package(va REQUIRED)
find_package(va-drm REQUIRED)
find_package(OpenCL REQUIRED)

add_compile_definitions(CL_TARGET_OPENCL_VERSION=120)

# Build our NUClear module
nuclear_module(
  INCLUDES ${TurboJPEG_INCLUDE_DIRS} ${OpenCL_INCLUDE_DIRS} ${va_INCLUDE_DIRS} ${va-drm_INCLUDE_DIRS}
  LIBRARIES ${TurboJPEG_LIBRARIES} ${OpenCL_LIBRARIES} ${va_LIBRARIES} ${va-drm_LIBRARIES}
)
