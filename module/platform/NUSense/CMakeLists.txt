# STM32H7xx/H742V(G-I)(H-T)_H743V(G-I)(H-T)_H750VBT_H753VI(H-T)

include(ExternalProject)
set(firmware_src_dir "${CMAKE_CURRENT_SOURCE_DIR}/firmware")
set(firmware_binary_dir "${firmware_binary_dir}")

ExternalProject_Add(
  nusense
  SOURCE_DIR ${firmware_src_dir}
  CMAKE_ARGS -DCMAKE_TOOLCHAIN_FILE="${firmware_src_dir}/cmake/Arduino/Arduino-toolchain.cmake"
             -DARDUINO_BOARD=stm32.GenH7 -DARDUINO_STM32_GENH7_MENU_PNUM_GENERIC_H753VITX=True
             -DARDUINO_STM32_GENH7_MENU_USB_CDC=ON -DARDUINO_STM32_GENH7_MENU_XUSB_HS=ON
  BINARY_DIR ${firmware_binary_dir}
  INSTALL_COMMAND ""
  BUILD_BYPRODUCTS ${firmware_binary_dir}/nusense.elf ${firmware_binary_dir}/nusense.hex
  BUILD_ALWAYS TRUE
  CONFIGURE_HANDLED_BY_BUILD TRUE
)

ExternalProject_Get_Property(nusense BINARY_DIR)

# Build our NUClear module
nuclear_module(DATA_FILES "${BINARY_DIR}/nusense.hex:firmware")
