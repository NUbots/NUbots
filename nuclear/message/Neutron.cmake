# Silence warnings about DEPFILE behaviour, we want the new behaviour
cmake_policy(SET CMP0116 NEW)

# Set the path to our generating scripts
set(script_source "${CMAKE_CURRENT_SOURCE_DIR}/../cmake/Scripts")

# Get the path to our parent directory above the message folder
get_filename_component(message_parent_dir "${PROJECT_SOURCE_DIR}/${NUCLEAR_MESSAGE_DIR}/.." ABSOLUTE)

# Generate the protocol buffers that are built into NUClear
set(builtin_dir "${CMAKE_CURRENT_SOURCE_DIR}/proto")
set(message_dir "${PROJECT_SOURCE_DIR}/${NUCLEAR_MESSAGE_DIR}")
file(GLOB_RECURSE builtin_protobufs "${builtin_dir}/**.proto")
file(GLOB_RECURSE message_protobufs "${message_dir}/**.proto")

# Locations to store each of the output components
set(pb_out "${CMAKE_CURRENT_BINARY_DIR}/protobuf")
set(nt_out "${CMAKE_CURRENT_BINARY_DIR}/neutron")
set(py_out "${CMAKE_CURRENT_BINARY_DIR}/python")
set(dep_out "${CMAKE_CURRENT_BINARY_DIR}/dependencies")

# We need protobuf and python to generate the neutron messages
find_package(Protobuf REQUIRED)
find_package(PythonInterp 3 REQUIRED)

# We need eigen for neutron messages
find_package(Eigen3 REQUIRED)

# Build the builtin protocol buffers as normal
foreach(proto ${builtin_protobufs})

  get_filename_component(file_we ${proto} NAME_WE)

  add_custom_command(
    OUTPUT ${pb_out}/${file_we}.pb.cc ${pb_out}/${file_we}.pb.h ${py_out}/${file_we}_pb2.py ${dep_out}/${file_we}.d
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} ARGS --cpp_out=lite:${pb_out} --python_out=${py_out}
            --dependency_out=${dep_out}/${file_we}.d -I${builtin_dir} ${CMAKE_CURRENT_SOURCE_DIR}/proto/${file_we}.proto
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/proto/${file_we}.proto
    DEPFILE ${dep_out}/${file_we}.d
    COMMENT "Compiling protocol buffer ${proto}"
  )

  list(APPEND protobuf_src ${pb_out}/${file_we}.pb.cc ${pb_out}/${file_we}.pb.h)
  list(APPEND python_src ${py_out}/${file_we}_pb2.py)

endforeach(proto ${builtin_protobufs})
add_custom_target(nuclear_message_builtins DEPENDS ${protobuf_src} ${python_src})

# Build the user protocol buffers
include(GenerateNeutron)
foreach(proto ${message_protobufs})
  GenerateNeutron(
    PROTO "${proto}"
    PARENT_DIR "${message_parent_dir}"
    BUILTIN_DIR "${builtin_dir}"
    BUILTIN_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  )

  # Compute the generated target name
  file(RELATIVE_PATH output_path ${message_parent_dir} ${proto})
  get_filename_component(output_path ${output_path} DIRECTORY)
  get_filename_component(file_we ${proto} NAME_WE)
  string(REPLACE "/" "_" neutron_target "${output_path}_${file_we}_neutron")
  string(REGEX REPLACE "^_([a-zA-Z0-9_]+)$" "\\1" neutron_target ${neutron_target})

  # Add to the respective outputs
  list(APPEND protobuf_src $<TARGET_PROPERTY:${neutron_target},NEUTRON_PROTOBUF_SOURCE>)
  list(APPEND neutron_src $<TARGET_PROPERTY:${neutron_target},NEUTRON_CPP_SOURCE>)
  list(APPEND python_src $<TARGET_PROPERTY:${neutron_target},NEUTRON_PYTHON_SOURCE>)
endforeach(proto ${message_protobufs})

# Build the reflection header
add_custom_command(
  OUTPUT ${nt_out}/message/reflection.hpp
  COMMAND ${PYTHON_EXECUTABLE} ARGS ${script_source}/build_message_reflection.py ${py_out}
          ${nt_out}/message/reflection.hpp
  WORKING_DIRECTORY ${nt_out}
  DEPENDS ${python_src} ${script_source}/build_message_reflection.py
  COMMENT "Building the message reflection header"
)
list(APPEND neutron_src ${nt_out}/message/reflection.hpp)

# * Make this library be a system include when it's linked into other libraries
# * This will prevent clang-tidy from looking at the headers
add_library(nuclear_message_protobuf OBJECT ${protobuf_src})
set_target_properties(nuclear_message_protobuf PROPERTIES CXX_CLANG_TIDY "")
target_include_directories(nuclear_message_protobuf PRIVATE ${pb_out})
target_include_directories(nuclear_message_protobuf SYSTEM INTERFACE ${pb_out})
target_link_libraries(nuclear_message_protobuf protobuf::libprotobuf)

add_library(nuclear_message ${NUCLEAR_LINK_TYPE} ${neutron_src})
target_compile_features(nuclear_message PUBLIC cxx_std_17)
target_link_libraries(nuclear_message PUBLIC nuclear_message_protobuf)
target_link_libraries(nuclear_message PUBLIC Eigen3::Eigen)
target_include_directories(nuclear_message PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(nuclear_message PUBLIC ${nt_out})

# Generate in the lib folder so it gets installed
if(NUCLEAR_LINK_TYPE STREQUAL "SHARED")
  set_property(TARGET nuclear_message PROPERTY LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin/lib)
endif()

# Alias to the namespaced version
add_library(nuclear::message ALIAS nuclear_message)
