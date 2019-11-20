# Get the relative path to our message directory
get_filename_component(message_include_dir "${PROJECT_SOURCE_DIR}/${NUCLEAR_MESSAGE_DIR}/.." ABSOLUTE)
file(RELATIVE_PATH message_include_dir ${PROJECT_SOURCE_DIR} ${message_include_dir})

# Get our two include directories for message
set(message_source_include_dir "${PROJECT_SOURCE_DIR}/${message_include_dir}")
set(message_binary_include_dir "${PROJECT_BINARY_DIR}/${message_include_dir}")

# Make our message include directories variable
set(NUCLEAR_MESSAGE_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include ${message_source_include_dir} ${message_binary_include_dir}
    CACHE INTERNAL "Include directories for the message folder and generated sources"
)

# Include our message directories
include_directories(${NUCLEAR_MESSAGE_INCLUDE_DIRS})

# Get our source and binary directories for message
set(message_source_dir "${PROJECT_SOURCE_DIR}/${NUCLEAR_MESSAGE_DIR}")
set(message_binary_dir "${PROJECT_BINARY_DIR}/${NUCLEAR_MESSAGE_DIR}")

# We need protobuf and python to generate the neutron messages
find_package(Protobuf REQUIRED)
find_package(PythonInterp 3 REQUIRED)

include_directories(SYSTEM ${Protobuf_INCLUDE_DIRS})

# If we have the package pybind11 we can use to go generate python bindings FIND_PACKAGE(pybind11)

# We need Eigen3
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${Eigen3_INCLUDE_DIRS})

# We need Eigen3
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${Eigen3_INCLUDE_DIRS})

# If we found pybind11 include its directories
if(pybind11_FOUND)
  find_package(PythonLibsNew 3 REQUIRED)

  include_directories(SYSTEM ${pybind11_INCLUDE_DIRS})
  include_directories(SYSTEM ${PYTHON_INCLUDE_DIRS})
endif()

# Build our builtin protobuf classes
file(GLOB_RECURSE builtin "${CMAKE_CURRENT_SOURCE_DIR}/proto/**.proto")
foreach(proto ${builtin})

  # Get the file without the extension
  get_filename_component(file_we ${proto} NAME_WE)

  # Run the protocol buffer compiler on the builtin protocol buffers
  add_custom_command(
    OUTPUT "${message_binary_include_dir}/${file_we}.pb.cc" "${message_binary_include_dir}/${file_we}.pb.h"
           "${message_binary_include_dir}/${file_we}_pb2.py"
    COMMAND
      ${PROTOBUF_PROTOC_EXECUTABLE} ARGS --cpp_out=lite:${message_binary_include_dir}
      --python_out=${message_binary_include_dir} -I${CMAKE_CURRENT_SOURCE_DIR}/proto
      "${CMAKE_CURRENT_SOURCE_DIR}/proto/${file_we}.proto"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/proto/${file_we}.proto"
    COMMENT "Compiling protocol buffer ${proto}"
  )

  set(src ${src} "${message_binary_include_dir}/${file_we}.pb.cc" "${message_binary_include_dir}/${file_we}.pb.h"
          "${message_binary_include_dir}/${file_we}_pb2.py"
  )

  # Prevent Effective C++ and unused parameter error checks being performed on generated files.
  set_source_files_properties(
    "${message_binary_include_dir}/${file_we}.pb"
    "${message_binary_include_dir}/${file_we}.proto"
    "${message_binary_include_dir}/${file_we}.pb.cc"
    "${message_binary_include_dir}/${file_we}.pb.h"
    "${message_binary_include_dir}/${file_we}.cpp"
    "${message_binary_include_dir}/${file_we}.py.cpp"
    "${message_binary_include_dir}/${file_we}.h"
    "${message_binary_include_dir}/${file_we}_pb2.py"
    PROPERTIES
    COMPILE_FLAGS
    "-Wno-unused-parameter -Wno-error=unused-parameter -Wno-error"
  )

endforeach(proto)

# Get our dependency files for our message class generator
file(GLOB_RECURSE message_class_generator_depends "${CMAKE_CURRENT_SOURCE_DIR}/generator/**.py")

unset(message_dependencies)

# Build all of our normal messages
file(GLOB_RECURSE protobufs "${message_source_dir}/**.proto")
foreach(proto ${protobufs})

  # Get the file without the extension
  get_filename_component(file_we ${proto} NAME_WE)

  # Calculate the Output Directory
  file(RELATIVE_PATH message_rel_path ${message_source_dir} ${proto})
  get_filename_component(outputpath ${message_rel_path} PATH)
  set(outputpath "${message_binary_dir}/${outputpath}")

  # Create the output directory
  file(MAKE_DIRECTORY ${outputpath})

  # Get the dependencies on this protobuf so we can recompile on changes If they change you will need to re cmake
  execute_process(
    COMMAND
      ${PROTOBUF_PROTOC_EXECUTABLE} --dependency_out=${CMAKE_CURRENT_BINARY_DIR}/temp1
      --descriptor_set_out=${CMAKE_CURRENT_BINARY_DIR}/temp2 -I${message_source_include_dir}
      -I${CMAKE_CURRENT_SOURCE_DIR}/proto ${proto}
  )

  file(READ "${CMAKE_CURRENT_BINARY_DIR}/temp1" dependencies)
  string(REGEX REPLACE "\\\\\n" ";" dependencies ${dependencies})
  file(REMOVE "${CMAKE_CURRENT_BINARY_DIR}/temp1" "${CMAKE_CURRENT_BINARY_DIR}/temp2")

  unset(source_depends)
  unset(binary_depends)

  # Clean our dependency list
  foreach(depend ${dependencies})
    string(STRIP ${depend} depend)
    file(RELATIVE_PATH depend_rel ${message_source_dir} ${depend})

    # Absolute dependencies
    if(depend_rel MATCHES "^\\.\\.")
      set(source_depends ${source_depends} ${depend})
      set(binary_depends ${binary_depends} ${depend})
      # Relative dependencies
    else()
      set(source_depends ${source_depends} "${message_source_dir}/${depend_rel}")
      set(binary_depends ${binary_depends} "${message_binary_dir}/${depend_rel}")
    endif()
  endforeach()

  # Extract the protocol buffer information so we can generate code off it
  add_custom_command(
    OUTPUT "${outputpath}/${file_we}.pb" "${outputpath}/${file_we}.dep"
    COMMAND
      ${PROTOBUF_PROTOC_EXECUTABLE} ARGS --descriptor_set_out="${outputpath}/${file_we}.pb"
      --dependency_out="${outputpath}/${file_we}.dep" -I${message_source_include_dir}
      -I${CMAKE_CURRENT_SOURCE_DIR}/proto ${proto}
    DEPENDS ${source_depends}
    COMMENT "Extracting protocol buffer information from ${proto}"
  )

  # Gather the dependencies for each message.
  set(message_dependencies ${message_dependencies} "${outputpath}/${file_we}.dep")

  # Repackage our protocol buffers so they don't collide with the actual classes when we make our c++ protobuf classes
  # by adding protobuf to the package
  add_custom_command(
    OUTPUT "${outputpath}/${file_we}.proto"
    COMMAND ${PYTHON_EXECUTABLE} ARGS "${CMAKE_CURRENT_SOURCE_DIR}/repackage_message.py" ${proto} ${outputpath}
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/repackage_message.py" ${source_depends}
    COMMENT "Repackaging protobuf ${proto}"
  )

  # Run the protocol buffer compiler on these new protobufs
  add_custom_command(
    OUTPUT "${outputpath}/${file_we}.pb.cc" "${outputpath}/${file_we}.pb.h" "${outputpath}/${file_we}_pb2.py"
    COMMAND
      ${PROTOBUF_PROTOC_EXECUTABLE} ARGS --cpp_out=lite:${message_binary_include_dir}
      --python_out=${message_binary_include_dir} -I${message_binary_include_dir} -I${CMAKE_CURRENT_SOURCE_DIR}/proto
      "${outputpath}/${file_we}.proto"
    DEPENDS ${binary_depends}
    COMMENT "Compiling protocol buffer ${proto}"
  )

  # Build our c++ class from the extracted information
  add_custom_command(
    OUTPUT "${outputpath}/${file_we}.cpp" "${outputpath}/${file_we}.py.cpp" "${outputpath}/${file_we}.h"
    COMMAND ${PYTHON_EXECUTABLE} ARGS "${CMAKE_CURRENT_SOURCE_DIR}/build_message_class.py" "${outputpath}/${file_we}"
            ${outputpath}
    WORKING_DIRECTORY ${message_binary_dir}
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/build_message_class.py" ${message_class_generator_depends}
            "${message_binary_include_dir}/Neutron_pb2.py" "${outputpath}/${file_we}.pb"
    COMMENT "Building classes for ${proto}"
  )

  # The protobuf descriptions are generated
  set_source_files_properties(
    "${outputpath}/${file_we}.pb"
    "${outputpath}/${file_we}.proto"
    "${outputpath}/${file_we}.pb.cc"
    "${outputpath}/${file_we}.pb.h"
    "${outputpath}/${file_we}.cpp"
    "${outputpath}/${file_we}.py.cpp"
    "${outputpath}/${file_we}.h"
    "${outputpath}/${file_we}_pb2.py"
    PROPERTIES
    GENERATED
    TRUE
    # Prevent Effective C++ and unused parameter error checks being performed on generated files.
    COMPILE_FLAGS
    "-Wno-unused-parameter -Wno-error=unused-parameter -Wno-error"
  )

  # Add the generated files to our list
  set(src ${src} "${outputpath}/${file_we}.pb.cc" "${outputpath}/${file_we}.pb.h" "${outputpath}/${file_we}.cpp"
          "${outputpath}/${file_we}.h"
  )

  # If we have pybind11 also add the python bindings
  if(pybind11_FOUND)
    set(src ${src} "${outputpath}/${file_we}.py.cpp")
  endif()

endforeach(proto)

# If we have pybind11 we need to generate our final binding class
if(pybind11_FOUND)
  # Build our outer python binding wrapper class
  add_custom_command(
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/outer_python_binding.cpp"
    COMMAND
      ${PYTHON_EXECUTABLE} ARGS "${CMAKE_CURRENT_SOURCE_DIR}/build_outer_python_binding.py"
      "${CMAKE_CURRENT_BINARY_DIR}/outer_python_binding.cpp" "${PROJECT_SOURCE_DIR}/${NUCLEAR_MESSAGE_DIR}"
      ${message_dependencies}
    WORKING_DIRECTORY ${message_binary_dir}
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/build_outer_python_binding.py" ${message_dependencies}
    COMMENT "Building outer python message binding"
  )

  set(src ${src} "${CMAKE_CURRENT_BINARY_DIR}/outer_python_binding.cpp")
endif()

if(src)
  # Build a library from these files
  if(NUCLEAR_SHARED_BUILD)
    add_library(nuclear_message SHARED ${protobufs} ${src})
    set_property(TARGET nuclear_message PROPERTY LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin/lib")
  else()
    add_library(nuclear_message STATIC ${protobufs} ${src})
  endif()

  # The library uses protocol buffers
  target_link_libraries(nuclear_message ${PROTOBUF_LIBRARIES})
  target_link_libraries(nuclear_message ${NUClear_LIBRARIES})

  # If we have pybind11 we need to make this a python library too
  if(pybind11_FOUND)
    target_link_libraries(nuclear_message ${PYTHON_LIBRARIES})

    # Work out what python expects the name of the library to be
    set(python_module_path "${PYTHON_MODULE_PREFIX}message${PYTHON_MODULE_EXTENSION}")

    # Make our NUClear python directory for including
    file(MAKE_DIRECTORY "${PROJECT_BINARY_DIR}/python/nuclear")

    # Create symlinks to the files
    add_custom_command(
      TARGET nuclear_message POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:nuclear_message>
              "${PROJECT_BINARY_DIR}/python/nuclear/${python_module_path}"
      COMMENT "Copying messages lib into python file format"
    )

    add_custom_command(
      TARGET nuclear_message POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy "${NUCLEAR_ROLES_DIR}/module/python/nuclear.py"
              "${PROJECT_BINARY_DIR}/python/nuclear/nuclear.py"
      DEPENDS "${NUCLEAR_ROLES_DIR}/module/python/nuclear.py"
      COMMENT "Copying nuclear.py to python build directory"
    )

  endif()

  # Add to our list of NUClear message libraries
  set(NUCLEAR_MESSAGE_LIBRARIES
      nuclear_message
      CACHE INTERNAL "List of libraries that are built as messages" FORCE
  )

  # Put it in an IDE group for shared
  set_property(TARGET nuclear_message PROPERTY FOLDER "shared/")
endif()
