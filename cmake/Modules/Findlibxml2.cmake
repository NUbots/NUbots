include(ToolchainLibraryFinder)

find_package(lzma)

ToolchainLibraryFinder(
  NAME libxml2
  HEADER libxml2/libxml/
  LIBRARY xml2
)

target_link_libraries(libxml2::libxml2 INTERFACE lzma::lzma)
