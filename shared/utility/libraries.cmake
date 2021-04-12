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
endif()

# Generate the decoders for each of the message types we know
add_custom_command(
  OUTPUT "${source_file}"
  COMMAND ${PYTHON_EXECUTABLE} ARGS "${CMAKE_CURRENT_SOURCE_DIR}/src/generate_dataplayback.py"
          "${PROJECT_BINARY_DIR}/nuclear/message/python" "${source_file}"
  DEPENDS nuclear::message "${CMAKE_CURRENT_SOURCE_DIR}/src/generate_dataplayback.py"
  COMMENT "Generating DataPlayback system for current messages"
)

target_compile_features(nuclear_utility PUBLIC cxx_std_17)
