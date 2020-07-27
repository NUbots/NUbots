# Generate our data logging module to handle all the message types
find_package(PythonInterp 3 REQUIRED)

set(source_file "${CMAKE_CURRENT_BINARY_DIR}/src/register_players.cpp")

# Generate the decoders for each of the message types we know
add_custom_command(
  OUTPUT "${source_file}"
  COMMAND ${PYTHON_EXECUTABLE} ARGS "${CMAKE_CURRENT_SOURCE_DIR}/src/generate_dataplayback.py"
          "${PROJECT_BINARY_DIR}/shared" "${source_file}"
  DEPENDS ${NUCLEAR_MESSAGE_LIBRARIES} "${CMAKE_CURRENT_SOURCE_DIR}/src/generate_dataplayback.py"
  COMMENT "Generating DataPlayback system for current messages"
)

# Create a symlink from to recordings so we can access them from build (helpful for docker)
add_custom_target(
  recordings_link ALL
  COMMAND ${CMAKE_COMMAND} -E create_symlink "${CMAKE_SOURCE_DIR}/recordings" ${CMAKE_BINARY_DIR}/recordings
  COMMENT "Creating a link to the recordings directory"
)

# Build our NUClear module
nuclear_module(SOURCES "${source_file}")
