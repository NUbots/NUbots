# Generate our data logging module to handle all the message types
find_package(PythonInterp 3 REQUIRED)

set(source_file "${CMAKE_CURRENT_BINARY_DIR}/src/message_handles.cpp")

# Generate the encoders for different message types
add_custom_command(
  USES_TERMINAL
  OUTPUT "${source_file}" "${config_file}"
  COMMAND ${PYTHON_EXECUTABLE} ARGS "${CMAKE_CURRENT_SOURCE_DIR}/src/generate_datalogging.py"
          "${PROJECT_BINARY_DIR}/shared" "${source_file}"
  DEPENDS ${NUCLEAR_MESSAGE_LIBRARIES} "${CMAKE_CURRENT_SOURCE_DIR}/src/generate_datalogging.py"
  COMMENT "Generating DataLogging system for current messages"
)

# Build our NUClear module
nuclear_module(SOURCES "${source_file}")
