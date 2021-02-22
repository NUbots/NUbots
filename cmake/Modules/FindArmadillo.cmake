# Find ALSA for eSpeak
include(ToolchainLibraryFinder)

# OpenBLAS for armadillo
find_package(OpenBLAS REQUIRED)

ToolchainLibraryFinder(
  NAME Armadillo
  HEADER armadillo
  LIBRARY armadillo
)

target_link_libraries(Armadillo::Armadillo INTERFACE OpenBLAS::OpenBLAS)
