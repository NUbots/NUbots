find_package(Eigen3 REQUIRED)
find_package(ExprTk REQUIRED)
find_package(fmt REQUIRED)
find_package(Aravis REQUIRED)
find_package(glib2 REQUIRED)

target_link_libraries(nuclear_utility PRIVATE ExprTk::ExprTk)
target_link_libraries(nuclear_utility PUBLIC Eigen3::Eigen)
target_link_libraries(nuclear_utility PUBLIC fmt::fmt)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  find_package(libbacktrace REQUIRED)
  target_link_libraries(nuclear_utility PUBLIC libbacktrace::libbacktrace ${CMAKE_DL_LIBS})
endif(CMAKE_BUILD_TYPE STREQUAL "Debug")

target_compile_features(nuclear_utility PUBLIC cxx_std_17)
