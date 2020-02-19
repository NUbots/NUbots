# Generate our data logging module to handle all the message types
find_package(PythonInterp 3 REQUIRED)

set(source_file "${CMAKE_CURRENT_BINARY_DIR}/src/DataPlayback.cpp")
set(config_file "${CMAKE_CURRENT_BINARY_DIR}/data/config/DataPlayback.yaml")

# Copy across the files
add_custom_command(
  USES_TERMINAL
  OUTPUT "${source_file}" "${config_file}"
  COMMAND ${PYTHON_EXECUTABLE} ARGS "${CMAKE_CURRENT_SOURCE_DIR}/src/generate_dataplayback.py"
          "${PROJECT_BINARY_DIR}/shared" "${source_file}" "${config_file}"
  DEPENDS ${NUCLEAR_MESSAGE_LIBRARIES} "${CMAKE_CURRENT_SOURCE_DIR}/src/generate_dataplayback.py"
  COMMENT "Generating DataPlayback system for current messages"
)

# Build our NUClear module
nuclear_module(SOURCES "${source_file}" DATA_FILES "${config_file}:config")
