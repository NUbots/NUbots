include(ToolchainLibraryFinder)

find_package(glib2 REQUIRED)
find_package(libusb1 REQUIRED)
find_package(ffi REQUIRED)
find_package(libxml2 REQUIRED)

ToolchainLibraryFinder(
  NAME Aravis
  HEADER arv.h
  LIBRARY aravis-0.8
  PATH_SUFFIX aravis-0.8
)

target_link_libraries(Aravis::Aravis INTERFACE glib2::glib2)
target_link_libraries(Aravis::Aravis INTERFACE libusb1::libusb1)
target_link_libraries(Aravis::Aravis INTERFACE ffi::ffi)
target_link_libraries(Aravis::Aravis INTERFACE libxml2::libxml2)
