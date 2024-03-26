# ######################################################################################################################
# TOOLCHAIN BLOCK - This block must be first in the file #
# ######################################################################################################################

cmake_minimum_required(VERSION 3.12.0)

# Select the correct microcontroller and variant
set(ARDUINO_BOARD stm32.GenH7)
set(ARDUINO_STM32_GENH7_MENU_PNUM_GENERIC_H753VITX ON)

# Manage serial ports ourselves
set(ARDUINO_STM32_GENH7_MENU_XSERIAL_NONE ON)

# Enable USB in CDC mode in USB2.0 (HS) mode
# Creates the `USBSerial` object
set(ARDUINO_STM32_GENH7_MENU_USB_CDC ON)
set(ARDUINO_STM32_GENH7_MENU_XUSB_HS ON)

# Optimise with LTO with a full c library
set(ARDUINO_STM32_GENH7_MENU_OPT_OSLTO ON)
set(ARDUINO_STM32_GENH7_MENU_RTLIB_FULL ON)

# For some reason the STM toolchain expects a file in sketch/build.opt
make_directory(${CMAKE_CURRENT_BINARY_DIR}/sketch)
file(TOUCH ${CMAKE_CURRENT_BINARY_DIR}/sketch/build.opt)

# Initialise the Arduino toolchain
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Arduino/Arduino-toolchain.cmake")
include(${CMAKE_TOOLCHAIN_FILE})

# ######################################################################################################################
project(NUSense CXX)

# Always MinSizeRel for the firmware
set(CMAKE_BUILD_TYPE MinSizeRel)

# Turn on all the warnings
add_compile_options(-Wall -Wpedantic -Wextra)

# Turn on coloured diagnostics
add_compile_options(-fdiagnostics-color=always)

# Glob all the source files
file(GLOB src "src/*.cpp" "src/**/*.cpp" "src/*.hpp" "src/*/**.hpp")

# run generate.py to generate the git version file
find_package(PythonInterp 3 REQUIRED)
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/git_version.hpp
  COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/generate_git_version.py ${CMAKE_CURRENT_SOURCE_DIR}
          ${CMAKE_CURRENT_BINARY_DIR}/git_version.hpp
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/generate_git_version.py ${src}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Generating git version file"
)
list(APPEND src ${CMAKE_CURRENT_BINARY_DIR}/git_version.hpp)

add_executable(nusense ${src})

target_include_directories(nusense PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(nusense PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# Link in the arduino core library and SrcWrapper to get the appropriate HAL
target_link_arduino_libraries(nusense PRIVATE core)
target_link_arduino_libraries(nusense PRIVATE SrcWrapper)

# Enable generation of the hex file
target_enable_arduino_upload(nusense)
