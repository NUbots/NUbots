find_package(Eigen3 REQUIRED)
target_link_libraries(nuclear_utility PUBLIC Eigen3::Eigen)

find_package(YAML-CPP REQUIRED)
target_link_libraries(nuclear_utility PUBLIC YAML-CPP::YAML-CPP)

find_package(ExprTk REQUIRED)
target_link_libraries(nuclear_utility PRIVATE ExprTk::ExprTk)

find_package(fmt REQUIRED)
target_link_libraries(nuclear_utility PUBLIC fmt::fmt)

find_package(Aravis REQUIRED)
target_link_libraries(nuclear_utility PUBLIC Aravis::Aravis)

find_package(Armadillo REQUIRED)
target_link_libraries(nuclear_utility PUBLIC Armadillo::Armadillo)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  find_package(libbacktrace REQUIRED)
  target_link_libraries(nuclear_utility PUBLIC libbacktrace::libbacktrace ${CMAKE_DL_LIBS})
endif(CMAKE_BUILD_TYPE STREQUAL "Debug")

target_compile_features(nuclear_utility PUBLIC cxx_std_17)
