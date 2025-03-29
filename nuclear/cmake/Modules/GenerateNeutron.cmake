define_property(
  TARGET
  PROPERTY NEUTRON_CPP_SOURCE
  BRIEF_DOCS "Generated neutron C++ source files"
  FULL_DOCS "Generated neutron C++ source files"
)
define_property(
  TARGET
  PROPERTY NEUTRON_PROTOBUF_SOURCE
  BRIEF_DOCS "Generated protobuf C/C++ source files"
  FULL_DOCS "Generated protobuf C/C++ source files"
)
define_property(
  TARGET
  PROPERTY NEUTRON_PYTHON_SOURCE
  BRIEF_DOCS "Generated protobuf python source files"
  FULL_DOCS "Generated protobuf python source files"
)

include(CMakeParseArguments)
function(GenerateNeutron)
  # We need protobuf and python to generate the neutron messages
  find_package(Protobuf REQUIRED)
  find_package(Python3 REQUIRED)

  # Set the path to our generating scripts
  set(SCRIPT_SOURCE "${PROJECT_SOURCE_DIR}/nuclear/cmake/Scripts")

  # Files that are used to generate the neutron files
  file(GLOB_RECURSE message_class_generator_files "${SCRIPT_SOURCE}/generator/**.py")

  # Extract the arguments from our function call
  set(options, "")
  set(oneValueArgs "PROTO" "PARENT_DIR" "BUILTIN_DIR" "BUILTIN_OUTPUT_DIR")
  set(multiValueArgs "")
  cmake_parse_arguments(NEUTRON "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  # Locations to store each of the output components
  set(pb_out "${CMAKE_CURRENT_BINARY_DIR}/protobuf")
  set(nt_out "${CMAKE_CURRENT_BINARY_DIR}/neutron")
  set(py_out "${CMAKE_CURRENT_BINARY_DIR}/python")

  # Extract the components of the filename that we need
  get_filename_component(file_we ${NEUTRON_PROTO} NAME_WE)
  file(RELATIVE_PATH output_path ${NEUTRON_PARENT_DIR} ${NEUTRON_PROTO})
  get_filename_component(output_path ${output_path} DIRECTORY)

  # Make sure paths are normalised
  cmake_path(SET NEUTRON_PROTO NORMALIZE "${NEUTRON_PROTO}")
  cmake_path(SET NEUTRON_PARENT_DIR NORMALIZE "${NEUTRON_PARENT_DIR}")
  cmake_path(SET NEUTRON_BUILTIN_DIR NORMALIZE "${NEUTRON_BUILTIN_DIR}")
  cmake_path(SET NEUTRON_BUILTIN_OUTPUT_DIR NORMALIZE "${NEUTRON_BUILTIN_OUTPUT_DIR}")
  cmake_path(SET pb NORMALIZE "${pb_out}/${output_path}/${file_we}")
  cmake_path(SET py NORMALIZE "${py_out}/${output_path}/${file_we}")
  cmake_path(SET nt NORMALIZE "${nt_out}/${output_path}/${file_we}")

  # Make sure the output paths exist
  file(MAKE_DIRECTORY "${pb_out}/${output_path}")
  file(MAKE_DIRECTORY "${nt_out}/${output_path}")
  file(MAKE_DIRECTORY "${py_out}/${output_path}")

  # Name of the target that will be created for this neutron set(neutron_target ${file_we}_neutron)
  string(REPLACE "/" "_" neutron_target "${output_path}_${file_we}_neutron")
  string(REGEX REPLACE "^_([a-zA-Z0-9_]+)$" "\\1" neutron_target ${neutron_target})

  #
  # DEPENDENCIES
  #
  # Ideally ninja would do this at runtime, but for now we have to work out what dependencies each of the protocol
  # buffer files have. If these change we just have to hope that it'll work until someone runs cmake again
  execute_process(
    COMMAND
      ${PROTOBUF_PROTOC_EXECUTABLE} --dependency_out=${CMAKE_CURRENT_BINARY_DIR}/dependencies.txt
      --descriptor_set_out=${CMAKE_CURRENT_BINARY_DIR}/descriptor.pb -I${NEUTRON_PARENT_DIR} -I${NEUTRON_BUILTIN_DIR}
      ${NEUTRON_PROTO}
  )
  file(READ "${CMAKE_CURRENT_BINARY_DIR}/dependencies.txt" dependencies)
  string(REGEX REPLACE "\\\\\n" ";" dependencies ${dependencies})
  file(REMOVE "${CMAKE_CURRENT_BINARY_DIR}/dependencies.txt" "${CMAKE_CURRENT_BINARY_DIR}/descriptor.pb")

  # Clean our dependency list
  foreach(depend ${dependencies})
    string(STRIP ${depend} depend)
    string(REGEX REPLACE "^[^:]*:[ \t\r\n]*" "" depend ${depend})
    file(RELATIVE_PATH depend_rel ${NEUTRON_PARENT_DIR} ${depend})

    # * Add a dependency to the target that generates the neutron for this dependency
    # * We specifically exclude adding dependencies for system protobufs (these have "/google/protobuf/" in their path)
    # * and NUClearRoles builtin protobufs (these have "/nuclear/message/proto/" in their path)
    get_filename_component(depend_we ${depend_rel} NAME_WE)
    get_filename_component(output_path ${depend_rel} DIRECTORY)
    string(REPLACE "/" "_" target_depend "${output_path}_${depend_we}_neutron")
    string(REGEX REPLACE "^_([a-zA-Z0-9_]+)$" "\\1" target_depend ${target_depend})

    string(FIND "${depend}" "/nuclear/message/proto/" is_builtin)
    string(FIND "${depend}" "/google/protobuf/" is_system)
    if(is_builtin EQUAL -1
       AND is_system EQUAL -1
       AND NOT neutron_target STREQUAL target_depend
    )
      list(APPEND target_depends ${target_depend})
    endif()

    if(depend_rel MATCHES "^\\.\\.")
      # Absolute dependencies
      cmake_path(SET depend NORMALIZE "${depend}")
      list(APPEND source_depends ${depend})
      list(APPEND binary_depends ${depend})
    else()
      # Relative dependencies
      list(APPEND source_depends ${NEUTRON_PARENT_DIR}/${depend_rel})
      list(APPEND binary_depends ${pb_out}/${depend_rel})
    endif()
  endforeach()

  #
  # PROTOCOL BUFFERS
  #
  # Repackage our protocol buffers so they don't collide with the actual classes when we make our c++ protobuf classes
  # by adding protobuf to the package
  add_custom_command(
    OUTPUT "${pb}.proto"
    COMMAND ${PYTHON_EXECUTABLE} ARGS "${SCRIPT_SOURCE}/repackage_message.py" "${NEUTRON_PROTO}" "${pb}.proto"
    WORKING_DIRECTORY "${SCRIPT_SOURCE}"
    DEPENDS "${SCRIPT_SOURCE}/repackage_message.py" ${NEUTRON_PROTO}
    COMMENT "Repackaging protobuf ${NEUTRON_PROTO}"
  )

  # Run the protocol buffer compiler on these new protobufs
  add_custom_command(
    OUTPUT "${pb}.pb.cc" "${pb}.pb.h" "${py}_pb2.py"
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} ARGS --cpp_out=lite:${pb_out} --python_out=${py_out} -I${pb_out}
            -I${NEUTRON_BUILTIN_DIR} "${pb}.proto"
    DEPENDS ${binary_depends} "${pb}.proto" ${target_depends}
    COMMENT "Compiling protocol buffer ${NEUTRON_PROTO}"
  )

  #
  # NEUTRONS
  #
  # Extract the protocol buffer information so we can generate code off it
  add_custom_command(
    OUTPUT "${nt}.pb"
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} ARGS --descriptor_set_out="${nt}.pb" -I${NEUTRON_PARENT_DIR}
            -I${NEUTRON_BUILTIN_DIR} ${NEUTRON_PROTO}
    DEPENDS ${source_depends}
    COMMENT "Extracting protocol buffer information from ${NEUTRON_PROTO}"
  )

  # Build our c++ class from the extracted information
  add_custom_command(
    OUTPUT "${nt}.cpp" "${nt}.py.cpp" "${nt}.hpp"
    COMMAND ${CMAKE_COMMAND} -E env NEUTRON_BUILTIN_DIR=${NEUTRON_BUILTIN_OUTPUT_DIR} ${PYTHON_EXECUTABLE} ARGS
            "${SCRIPT_SOURCE}/build_message_class.py" "${nt}"
    WORKING_DIRECTORY "${nt_out}"
    DEPENDS "${SCRIPT_SOURCE}/build_message_class.py" ${message_class_generator_files} "${nt}.pb"
            nuclear_message_builtins
    COMMENT "Building classes for ${NEUTRON_PROTO}"
  )

  # Create a target for people to depend on
  add_custom_target(${neutron_target} DEPENDS "${pb}.pb.cc" "${pb}.pb.h" "${nt}.cpp" "${nt}.hpp" "${py}_pb2.py")
  set_target_properties(
    ${neutron_target}
    PROPERTIES NEUTRON_CPP_SOURCE "${nt}.cpp;${nt}.hpp"
               NEUTRON_PROTOBUF_SOURCE "${pb}.pb.cc;${pb}.pb.h"
               NEUTRON_PYTHON_SOURCE "${py}_pb2.py"
  )

endfunction(GenerateNeutron)
