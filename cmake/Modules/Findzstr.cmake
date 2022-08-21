include(${PROJECT_SOURCE_DIR}/nuclear/cmake/Modules/HeaderLibrary.cmake)

find_package(zlib REQUIRED)

HeaderLibrary(
  NAME zstr
  HEADER "zstr/zstr.hpp"
  URL "https://raw.githubusercontent.com/mateidavid/zstr/v1.0.4/src/zstr.hpp"
)

target_link_libraries(zstr::zstr INTERFACE zlib::zlib)
