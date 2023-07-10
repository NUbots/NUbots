# Build the test message
include(GenerateNeutron)
GenerateNeutron(
  PROTO "${CMAKE_CURRENT_SOURCE_DIR}/MessageTest.proto"
  PARENT_DIR "${CMAKE_CURRENT_SOURCE_DIR}"
  BUILTIN_DIR "${PROJECT_SOURCE_DIR}/nuclear/message/proto"
  BUILTIN_OUTPUT_DIR "${PROJECT_BINARY_DIR}/nuclear/message"
)

# Compute the generated target name
file(RELATIVE_PATH output_path ${CMAKE_CURRENT_SOURCE_DIR} "${CMAKE_CURRENT_SOURCE_DIR}/MessageTest.proto")
get_filename_component(output_path ${output_path} DIRECTORY)
get_filename_component(file_we "${CMAKE_CURRENT_SOURCE_DIR}/MessageTest.proto" NAME_WE)
string(REPLACE "/" "_" neutron_target "${output_path}_${file_we}_neutron")
string(REGEX REPLACE "^_([a-zA-Z0-9_]+)$" "\\1" neutron_target ${neutron_target})

# * Make this library be a system include when it's linked into other libraries
# * This will prevent clang-tidy from looking at the headers
add_library(message_test_protobuf OBJECT $<TARGET_PROPERTY:${neutron_target},NEUTRON_PROTOBUF_SOURCE>)
set_target_properties(message_test_protobuf PROPERTIES CXX_CLANG_TIDY "")
target_include_directories(message_test_protobuf PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/protobuf")
target_include_directories(message_test_protobuf SYSTEM INTERFACE "${CMAKE_CURRENT_BINARY_DIR}/protobuf")
target_link_libraries(message_test_protobuf nuclear_message_protobuf)

target_sources(
  TestNUClearRoles PUBLIC $<TARGET_PROPERTY:${neutron_target},NEUTRON_CPP_SOURCE>
                          $<TARGET_OBJECTS:message_test_protobuf> "${CMAKE_CURRENT_SOURCE_DIR}/converter.cpp"
)
target_include_directories(TestNUClearRoles PUBLIC "${CMAKE_CURRENT_BINARY_DIR}")
target_include_directories(TestNUClearRoles SYSTEM PUBLIC "${CMAKE_CURRENT_BINARY_DIR}/protobuf")
