include(ToolchainLibraryFinder)

find_package(glib2 REQUIRED)

ToolchainLibraryFinder(
  NAME Aravis
  HEADER arv.h
  LIBRARY aravis-0.8
  PATH_SUFFIX aravis-0.8
)

target_link_libraries(Aravis::Aravis INTERFACE glib2::glib2)
