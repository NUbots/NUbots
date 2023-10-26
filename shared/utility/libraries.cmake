find_package(Eigen3 REQUIRED)
target_link_libraries(nuclear_utility PUBLIC Eigen3::Eigen)

find_package(yaml-cpp REQUIRED)
target_link_libraries(nuclear_utility PUBLIC yaml-cpp)

find_package(ExprTk REQUIRED)
target_link_libraries(nuclear_utility PRIVATE ExprTk::ExprTk)

find_package(fmt REQUIRED)
target_link_libraries(nuclear_utility PUBLIC fmt::fmt)

find_package(zstr REQUIRED)
target_link_libraries(nuclear_utility PUBLIC zstr::zstr)

find_package(mio REQUIRED)
target_link_libraries(nuclear_utility PUBLIC mio::mio)

find_package(Aravis REQUIRED)
target_link_libraries(nuclear_utility PUBLIC Aravis::Aravis)

find_package(tinyxml2 REQUIRED)
target_link_libraries(nuclear_utility PUBLIC tinyxml2::tinyxml2)

find_package(tinyrobotics REQUIRED)
target_link_libraries(nuclear_utility PUBLIC tinyrobotics::tinyrobotics)

find_package(NLopt REQUIRED)
target_link_libraries(nuclear_utility PUBLIC NLopt::nlopt)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  find_package(libbacktrace REQUIRED)
  target_link_libraries(nuclear_utility PUBLIC libbacktrace::libbacktrace ${CMAKE_DL_LIBS})
endif()

# Create a symlink to recordings so we can access them from build (helpful for docker)
add_custom_command(
  OUTPUT "${CMAKE_BINARY_DIR}/recordings"
  COMMAND ${CMAKE_COMMAND} -E create_symlink "${CMAKE_SOURCE_DIR}/recordings" "${CMAKE_BINARY_DIR}/recordings"
  COMMENT "Creating a link to the recordings directory"
)
target_sources(nuclear_utility PRIVATE "${CMAKE_BINARY_DIR}/recordings")

target_compile_features(nuclear_utility PUBLIC cxx_std_17)

# Add the scripts directory to the build directory
file(COPY "${PROJECT_SOURCE_DIR}/shared/utility/skill/scripts" DESTINATION ${PROJECT_BINARY_DIR})
# Add the scripts to the script files variable for the install script
file(GLOB_RECURSE scripts "${PROJECT_BINARY_DIR}/scripts/*")
set(SCRIPT_FILES
    ${scripts}
    CACHE INTERNAL "A list of all script files" FORCE
)

# Add the platform models directory to the build directory
file(COPY "${PROJECT_SOURCE_DIR}/shared/utility/platform/models" DESTINATION ${PROJECT_BINARY_DIR})
# Add the models to the model files variable for the install script
file(GLOB_RECURSE models "${PROJECT_BINARY_DIR}/models/*")
set(MODEL_FILES
    ${models}
    CACHE INTERNAL "A list of all model files" FORCE
)
