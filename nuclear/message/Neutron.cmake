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

# Files that are used to generate the neutron files
file(GLOB_RECURSE message_class_generator_files "${CMAKE_CURRENT_SOURCE_DIR}/generator/**.py")

# We need protobuf and python to generate the neutron messages
find_package(Protobuf REQUIRED)
find_package(PythonInterp 3 REQUIRED)

# We need Eigen for neutron messages
find_package(Eigen3 REQUIRED)

# We need pybind11 for python modules
find_package(pybind11 REQIRED)

# TODO(KipHamiltons): Not sure if I need these
find_package(PythonInterp 3 REQUIRED)
find_package(PythonLibsNew 3 REQUIRED)
# TODO(KipHamiltons): Not sure about these either. Try `target_include_directories` eventually, not including them at
# all
include_directories(SYSTEM ${pybind11_INCLUDE_DIRS})
include_directories(SYSTEM ${PYTHON_INCLUDE_DIRS})

# Build the builtin protocol buffers as normal
foreach(proto ${builtin_protobufs})

  get_filename_component(file_we ${proto} NAME_WE)

  add_custom_command(
    OUTPUT "${pb_out}/${file_we}.pb.cc" "${pb_out}/${file_we}.pb.h" "${py_out}/${file_we}_pb2.py"
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} ARGS --cpp_out=lite:${pb_out} --python_out=${py_out} -I${builtin_dir}
            "${CMAKE_CURRENT_SOURCE_DIR}/proto/${file_we}.proto"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/proto/${file_we}.proto"
    COMMENT "Compiling protocol buffer ${proto}"
  )

  list(APPEND protobuf_src "${pb_out}/${file_we}.pb.cc" "${pb_out}/${file_we}.pb.h")
  list(APPEND python_src "${py_out}/${file_we}_pb2.py")

  # Pybind11 C++ files TODO(KipHamiltons): Not sure this is actually doing anything
  list(APPEND neutron_src "${nt_out}/${file_we}.py.cpp")

endforeach(proto ${builtin_protobufs})

set(py_message_modules "")

# Build the user protocol buffers
foreach(proto ${message_protobufs})

  # Extract the components of the filename that we need
  get_filename_component(file_we ${proto} NAME_WE)
  file(RELATIVE_PATH output_path ${message_parent_dir} ${proto})
  get_filename_component(output_path ${output_path} PATH)
  set(pb ${pb_out}/${output_path}/${file_we})
  set(py ${py_out}/${output_path}/${file_we})
  set(nt ${nt_out}/${output_path}/${file_we})

  #
  # DEPENDENCIES
  #
  # Ideally ninja would do this at runtime, but for now we have to work out what dependencies each of the protocol
  # buffer files have. If these change we just have to hope that it'll work until someone runs cmake again
  execute_process(
    COMMAND
      ${PROTOBUF_PROTOC_EXECUTABLE} --dependency_out=${CMAKE_CURRENT_BINARY_DIR}/dependencies.txt
      --descriptor_set_out=${CMAKE_CURRENT_BINARY_DIR}/descriptor.pb -I${message_parent_dir} -I${builtin_dir} ${proto}
  )

  file(READ ${CMAKE_CURRENT_BINARY_DIR}/dependencies.txt dependencies)
  string(REGEX REPLACE "\\\\\n" ";" dependencies ${dependencies})
  file(REMOVE ${CMAKE_CURRENT_BINARY_DIR}/dependencies ${CMAKE_CURRENT_BINARY_DIR}/descriptor.pb)

  # Clean our dependency list
  unset(source_depends)
  unset(binary_depends)

  foreach(depend ${dependencies})
    string(STRIP ${depend} depend)
    file(RELATIVE_PATH depend_rel ${message_parent_dir} ${depend})

    # Absolute dependencies
    if(depend_rel MATCHES "^\\.\\.")
      list(APPEND source_depends ${depend})
      list(APPEND binary_depends ${depend})
      # Relative dependencies
    else()
      list(APPEND source_depends "${message_parent_dir}/${depend_rel}")
      list(APPEND binary_depends "${pb_out}/${depend_rel}")
    endif()
  endforeach()

  #
  # PROTOCOL BUFFERS
  #
  # Repackage our protocol buffers so they don't collide with the actual classes when we make our c++ protobuf classes
  # by adding protobuf to the package
  add_custom_command(
    OUTPUT "${pb}.proto"
    COMMAND ${PYTHON_EXECUTABLE} ARGS "${CMAKE_CURRENT_SOURCE_DIR}/repackage_message.py" "${proto}" "${pb}.proto"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/repackage_message.py" ${proto}
    COMMENT "Repackaging protobuf ${proto}"
  )

  # Run the protocol buffer compiler on these new protobufs
  add_custom_command(
    OUTPUT "${pb}.pb.cc" "${pb}.pb.h" "${py}_pb2.py"
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} ARGS --cpp_out=lite:${pb_out} --python_out=${py_out} -I${pb_out}
            -I${builtin_dir} "${pb}.proto"
    DEPENDS ${binary_depends} "${pb}.proto"
    COMMENT "Compiling protocol buffer ${proto}"
  )

  #
  # NEUTRONS
  #
  # Extract the protocol buffer information so we can generate code off it
  add_custom_command(
    OUTPUT "${nt}.pb"
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} ARGS --descriptor_set_out="${nt}.pb" -I${message_parent_dir} -I${builtin_dir}
            ${proto}
    DEPENDS ${source_depends} "${proto}"
    COMMENT "Extracting protocol buffer information from ${proto}"
  )

  # Build our c++ class from the extracted information
  add_custom_command(
    OUTPUT "${nt}.cpp" "${nt}.py.cpp" "${nt}.hpp"
    COMMAND ${PYTHON_EXECUTABLE} ARGS "${CMAKE_CURRENT_SOURCE_DIR}/build_message_class.py" "${nt}"
    WORKING_DIRECTORY "${nt_out}"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/build_message_class.py" ${message_class_generator_files} "${nt}.pb"
    COMMENT "Building classes for ${proto}"
  )

  # Add to the respective outputs
  list(APPEND protobuf_src "${pb}.pb.cc" "${pb}.pb.h")
  # TODO(KipHamiltons): Not sure if this is doing anything, adding ${nt}.py.cpp here
  list(APPEND neutron_src "${nt}.cpp" "${nt}.hpp" "${nt}.py.cpp")
  list(APPEND python_src "${py}_pb2.py")

  # TODO(KipHamiltons) verify this is required Add the message modules
  list(APPEND py_message_modules "${outputpath}/${file_we}_pb2.py")

endforeach(proto ${message_protobufs})

# * TODO(KipHamiltons) investigate if this is required * Add Neutron_pb2.py here to prevent adding duplicate items to
#   the list
list(APPEND python_src "${message_binary_include_dir}/Neutron_pb2.py")

# Build the reflection header
add_custom_command(
  OUTPUT "${nt_out}/message/reflection.hpp"
  COMMAND ${PYTHON_EXECUTABLE} ARGS "${CMAKE_CURRENT_SOURCE_DIR}/build_message_reflection.py" "${py_out}"
          "${nt_out}/message/reflection.hpp"
  WORKING_DIRECTORY "${nt_out}"
  DEPENDS ${python_src} "${CMAKE_CURRENT_SOURCE_DIR}/build_message_reflection.py"
  COMMENT "Building the message reflection header"
)
list(APPEND neutron_src "${nt_out}/message/reflection.hpp")

# Build our outer python binding wrapper class
add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/outer_python_binding.cpp"
  COMMAND
    ${PYTHON_EXECUTABLE} ARGS "${CMAKE_CURRENT_SOURCE_DIR}/build_outer_python_binding.py"
    "${CMAKE_CURRENT_BINARY_DIR}/outer_python_binding.cpp" "${PROJECT_SOURCE_DIR}/${NUCLEAR_MESSAGE_DIR}"
    ${dependencies}
  WORKING_DIRECTORY ${message_binary_dir}
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/build_outer_python_binding.py" ${dependencies}
  COMMENT "Building outer python message binding"
)

list(APPEND neutron_src "${CMAKE_CURRENT_BINARY_DIR}/outer_python_binding.cpp")

# Macro to list all subdirectories of a given directory
macro(SUBDIRLIST result curdir)
  # Get list of directory entries in current directory
  file(
    GLOB_RECURSE children
    LIST_DIRECTORIES TRUE
    RELATIVE ${curdir}
    ${curdir}/*
  )

  # Append all subdirectories of current directory to our directory list
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY "${curdir}/${child}")
      list(APPEND dirlist "${child}")
    endif(IS_DIRECTORY "${curdir}/${child}")
  endforeach(child)

  # Return result
  set(${result} ${dirlist})
endmacro(SUBDIRLIST)

# Generate a list of all of the python message files we will be generating This allows us to tell cmake at configure
# time what we will be generating at build time This should also allow us to set up proper dependencies and clean up
# generated files at clean time
set(py_messages "")
list(REMOVE_DUPLICATES py_message_modules)

subdirlist(py_messages ${message_source_dir})
list(APPEND py_messages "")
list(TRANSFORM py_messages PREPEND "${PROJECT_BINARY_DIR}/python/nuclear/message/")
list(TRANSFORM py_messages APPEND "/__init__.py")

# Generate module python file containing stub classes for all of our messages
add_custom_command(
  OUTPUT ${py_messages}
  BYPRODUCTS "${PROJECT_BINARY_DIR}/python/nuclear/messages.txt"
  COMMAND
    ${PYTHON_EXECUTABLE} ARGS "${CMAKE_CURRENT_SOURCE_DIR}/generate_python_messages.py" "${PROJECT_BINARY_DIR}/shared"
    "${PROJECT_BINARY_DIR}/python/nuclear" "${PROJECT_BINARY_DIR}/python/nuclear/messages.txt"
  WORKING_DIRECTORY ${message_binary_dir}
  DEPENDS ${src} ${py_message_modules}
  COMMENT "Generating python sub messages"
)

# Make sure all of the generated files are marked as generated
set_source_files_properties(${py_messages} PROPERTIES GENERATED TRUE)

# Create the python messages target and set the dependency chain up
add_custom_target(python_nuclear_message DEPENDS ${py_messages})
add_dependencies(nuclear_message python_nuclear_message)

# * Make this library be a system include when it's linked into other libraries
# * This will prevent clang-tidy from looking at the headers
add_library(nuclear_message_protobuf OBJECT ${protobuf_src})
set_target_properties(nuclear_message_protobuf PROPERTIES CXX_CLANG_TIDY "")
target_include_directories(nuclear_message_protobuf PRIVATE ${pb_out})
target_include_directories(nuclear_message_protobuf SYSTEM INTERFACE ${pb_out})
target_link_libraries(nuclear_message_protobuf protobuf::libprotobuf)

add_library(nuclear_message ${NUCLEAR_LINK_TYPE} ${neutron_src} ${py_message_modules})
target_compile_features(nuclear_message PUBLIC cxx_std_17)
target_link_libraries(nuclear_message PUBLIC nuclear_message_protobuf)
target_link_libraries(nuclear_message PUBLIC Eigen3::Eigen)
target_link_libraries(nuclear_message PUBLIC pybind11::pybind11)
# Don't know about this one
target_link_libraries(nuclear_message ${PYTHON_LIBRARIES})
target_include_directories(nuclear_message PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(nuclear_message PUBLIC ${nt_out})

# Generate in the lib folder so it gets installed
if(NUCLEAR_LINK_TYPE STREQUAL "SHARED")
  set_property(TARGET nuclear_message PROPERTY LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin/lib")
endif()

# Alias to the namespaced version
add_library(nuclear::message ALIAS nuclear_message)
