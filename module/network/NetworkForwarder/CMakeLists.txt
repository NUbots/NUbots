# Generate our handlers for all the message types
find_package(PythonInterp 3 REQUIRED)

set(source_file "${CMAKE_CURRENT_BINARY_DIR}/src/register_handles.cpp")

# Generate the decoders for each of the message types we know
add_custom_command(
  OUTPUT "${source_file}"
  COMMAND ${PYTHON_EXECUTABLE} ARGS "${CMAKE_CURRENT_SOURCE_DIR}/src/generate_handles.py"
          "${PROJECT_BINARY_DIR}/nuclear/message/python" "${source_file}"
  DEPENDS nuclear::message "${CMAKE_CURRENT_SOURCE_DIR}/src/generate_handles.py"
  COMMENT "Generating NetworkForwarder handles for all message types"
)

# Build our NUClear module
nuclear_module(SOURCES "${source_file}")
