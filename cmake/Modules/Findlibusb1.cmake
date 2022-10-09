include(ToolchainLibraryFinder)

find_package(libudev)

ToolchainLibraryFinder(
  NAME libusb1
  HEADER libusb-1.0/libusb.h
  LIBRARY usb-1.0
)

target_link_libraries(libusb1::libusb1 INTERFACE libudev::libudev)
