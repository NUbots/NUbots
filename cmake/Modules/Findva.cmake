include(ToolchainLibraryFinder)

find_package(va-drm REQUIRED)

ToolchainLibraryFinder(
  NAME va
  HEADER va/va.h
  LIBRARY va
)

target_link_libraries(va::va INTERFACE va-drm::va-drm)
