include(ExternalProject)
set(firmware_src_dir "${CMAKE_CURRENT_SOURCE_DIR}/firmware")
set(firmware_binary_dir "${firmware_binary_dir}")

ExternalProject_Add(
  nusense
  SOURCE_DIR ${firmware_src_dir}
  BINARY_DIR ${firmware_binary_dir}
  INSTALL_COMMAND ""
  BUILD_BYPRODUCTS ${firmware_binary_dir}/nusense.elf ${firmware_binary_dir}/nusense.hex
  BUILD_ALWAYS TRUE
  CONFIGURE_HANDLED_BY_BUILD TRUE
)

ExternalProject_Get_Property(nusense BINARY_DIR)

# Build our NUClear module
nuclear_module(DATA_FILES "${BINARY_DIR}/nusense.hex:firmware")
