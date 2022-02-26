include(ExternalProject)

# ######################################################################################################################
# CM730 Firmware builds for both Darwin and Igus models
# ######################################################################################################################

set(firmware_source "${CMAKE_CURRENT_BINARY_DIR}/CM730Firmware")
set(firmware_dir "${CMAKE_BINARY_DIR}/firmware")
set(bin_files "${firmware_source}/build/cm730_3cell.bin" "${firmware_source}/build/cm730_4cell.bin"
              "${firmware_source}/build/cm740_3cell.bin" "${firmware_source}/build/cm740_4cell.bin"
)

ExternalProject_Add(
  CM730Firmware
  DOWNLOAD_DIR "${firmware_source}"
  GIT_REPOSITORY "https://github.com/NUbots/CM730"
  GIT_TAG "master"
  PREFIX "${firmware_source}"
  UPDATE_COMMAND ""
  CMAKE_GENERATOR Ninja
  # CMAKE_ARGS -D -DCMAKE_TOOLCHAIN_FILE:STRING="${firmware_source}/CM730.cmake"
  BINARY_DIR "${firmware_source}/build"
  BUILD_COMMAND ninja
  BUILD_BYPRODUCTS ${bin_files}
                   # We have a custom installation step below.
  INSTALL_DIR "${firmware_source}/build"
  INSTALL_COMMAND ""
                  # We dont need to do any testing.
  TEST_COMMAND ""
)

# Build our NUClear module
foreach(bin_file ${bin_files})
  list(APPEND data_files "${bin_file}:firmware")
endforeach(bin_file)

nuclear_module(DATA_FILES ${data_files})
