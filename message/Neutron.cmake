# Get the relative path to our message directory
GET_FILENAME_COMPONENT(message_include_dir "${PROJECT_SOURCE_DIR}/${NUCLEAR_MESSAGE_DIR}/.." ABSOLUTE)
FILE(RELATIVE_PATH message_include_dir ${PROJECT_SOURCE_DIR} ${message_include_dir})

# Get our two include directories for message
SET(message_source_include_dir "${PROJECT_SOURCE_DIR}/${message_include_dir}")
SET(message_binary_include_dir "${PROJECT_BINARY_DIR}/${message_include_dir}")

# Make our message include directories variable
SET(NUCLEAR_MESSAGE_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${message_source_include_dir}
    ${message_binary_include_dir}
    CACHE INTERNAL "Include directories for the message folder and generated sources")

# Include our message directories
INCLUDE_DIRECTORIES(${NUCLEAR_MESSAGE_INCLUDE_DIRS})

# Get our source and binary directories for message
SET(message_source_dir "${PROJECT_SOURCE_DIR}/${NUCLEAR_MESSAGE_DIR}")
SET(message_binary_dir "${PROJECT_BINARY_DIR}/${NUCLEAR_MESSAGE_DIR}")

# We need protobuf and python to generate the neutron messages
FIND_PACKAGE(Protobuf REQUIRED)
FIND_PACKAGE(PythonInterp 3 REQUIRED)

INCLUDE_DIRECTORIES(SYSTEM ${Protobuf_INCLUDE_DIRS})

# If we have the package pybind11 we can use to go generate python bindings
FIND_PACKAGE(pybind11)

# We need Eigen3
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(SYSTEM ${Eigen3_INCLUDE_DIRS})

# If we found pybind11 include its directories
IF(pybind11_FOUND)
    FIND_PACKAGE(PythonLibsNew 3 REQUIRED)

    INCLUDE_DIRECTORIES(SYSTEM ${pybind11_INCLUDE_DIRS})
    INCLUDE_DIRECTORIES(SYSTEM ${PYTHON_INCLUDE_DIRS})
ENDIF()

# Build our builtin protobuf classes
FILE(GLOB_RECURSE builtin "${CMAKE_CURRENT_SOURCE_DIR}/proto/**.proto")
FOREACH(proto ${builtin})

    # Get the file without the extension
    GET_FILENAME_COMPONENT(file_we ${proto} NAME_WE)

    # Run the protocol buffer compiler on the builtin protocol buffers
    ADD_CUSTOM_COMMAND(
        OUTPUT "${message_binary_include_dir}/${file_we}.pb.cc"
               "${message_binary_include_dir}/${file_we}.pb.h"
               "${message_binary_include_dir}/${file_we}_pb2.py"
        COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
        ARGS --cpp_out=lite:${message_binary_include_dir}
             --python_out=${message_binary_include_dir}
             -I${CMAKE_CURRENT_SOURCE_DIR}/proto
             "${CMAKE_CURRENT_SOURCE_DIR}/proto/${file_we}.proto"
        DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/proto/${file_we}.proto"
        COMMENT "Compiling protocol buffer ${proto}")

    SET(src ${src}
            "${message_binary_include_dir}/${file_we}.pb.cc"
            "${message_binary_include_dir}/${file_we}.pb.h"
            "${message_binary_include_dir}/${file_we}_pb2.py")

    # Prevent Effective C++ and unused parameter error checks being performed on generated files.
    SET_SOURCE_FILES_PROPERTIES("${message_binary_include_dir}/${file_we}.pb"
                                "${message_binary_include_dir}/${file_we}.proto"
                                "${message_binary_include_dir}/${file_we}.pb.cc"
                                "${message_binary_include_dir}/${file_we}.pb.h"
                                "${message_binary_include_dir}/${file_we}.cpp"
                                "${message_binary_include_dir}/${file_we}.py.cpp"
                                "${message_binary_include_dir}/${file_we}.h"
                                 PROPERTIES COMPILE_FLAGS "-Wno-effc++ -Wno-error=unused-parameter -Wno-error")

ENDFOREACH(proto)

# Get our dependency files for our message class generator
FILE(GLOB_RECURSE message_class_generator_depends "${CMAKE_CURRENT_SOURCE_DIR}/generator/**.py")

UNSET(message_dependencies)

# Build all of our normal messages
FILE(GLOB_RECURSE protobufs "${message_source_dir}/**.proto")
FOREACH(proto ${protobufs})

    # Get the file without the extension
    GET_FILENAME_COMPONENT(file_we ${proto} NAME_WE)

    # Calculate the Output Directory
    FILE(RELATIVE_PATH message_rel_path ${message_source_dir} ${proto})
    GET_FILENAME_COMPONENT(outputpath ${message_rel_path} PATH)
    SET(outputpath "${message_binary_dir}/${outputpath}")

    # Create the output directory
    FILE(MAKE_DIRECTORY ${outputpath})

    # Get the dependencies on this protobuf so we can recompile on changes
    # If they change you will need to re cmake
    EXECUTE_PROCESS(COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
                            --dependency_out=${CMAKE_CURRENT_BINARY_DIR}/temp1
                            --descriptor_set_out=${CMAKE_CURRENT_BINARY_DIR}/temp2
                            -I${message_source_include_dir}
                            -I${CMAKE_CURRENT_SOURCE_DIR}/proto
                            ${proto})

    FILE(READ "${CMAKE_CURRENT_BINARY_DIR}/temp1" dependencies)
    STRING(REGEX REPLACE "\\\\\n" ";" dependencies ${dependencies})
    FILE(REMOVE "${CMAKE_CURRENT_BINARY_DIR}/temp1" "${CMAKE_CURRENT_BINARY_DIR}/temp2")

    UNSET(source_depends)
    UNSET(binary_depends)

    # Clean our dependency list
    FOREACH(depend ${dependencies})
        STRING(STRIP ${depend} depend)
        FILE(RELATIVE_PATH depend_rel ${message_source_dir} ${depend})

        # Absolute dependencies
        IF(depend_rel MATCHES "^\\.\\.")
            SET(source_depends ${source_depends} ${depend})
            SET(binary_depends ${binary_depends} ${depend})
        # Relative dependencies
        ELSE()
            SET(source_depends ${source_depends} "${message_source_dir}/${depend_rel}")
            SET(binary_depends ${binary_depends} "${message_binary_dir}/${depend_rel}")
        ENDIF()
    ENDFOREACH()

    # Extract the protocol buffer information so we can generate code off it
    ADD_CUSTOM_COMMAND(
        OUTPUT "${outputpath}/${file_we}.pb"
               "${outputpath}/${file_we}.dep"
        COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
        ARGS --descriptor_set_out="${outputpath}/${file_we}.pb"
             --dependency_out="${outputpath}/${file_we}.dep"
             -I${message_source_include_dir}
             -I${CMAKE_CURRENT_SOURCE_DIR}/proto
             ${proto}
        DEPENDS ${source_depends}
        COMMENT "Extracting protocol buffer information from ${proto}")

    # Gather the dependencies for each message.
    SET(message_dependencies ${message_dependencies} "${outputpath}/${file_we}.dep")

    # Repackage our protocol buffers so they don't collide with the actual classes
    # when we make our c++ protobuf classes by adding protobuf to the package
    ADD_CUSTOM_COMMAND(
        OUTPUT "${outputpath}/${file_we}.proto"
        COMMAND ${PYTHON_EXECUTABLE}
        ARGS "${CMAKE_CURRENT_SOURCE_DIR}/repackage_message.py" ${proto} ${outputpath}
        DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/repackage_message.py"
                ${source_depends}
        COMMENT "Repackaging protobuf ${proto}")

    # Run the protocol buffer compiler on these new protobufs
    ADD_CUSTOM_COMMAND(
        OUTPUT "${outputpath}/${file_we}.pb.cc"
               "${outputpath}/${file_we}.pb.h"
               "${outputpath}/${file_we}_pb2.py"
        COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
        ARGS --cpp_out=lite:${message_binary_include_dir}
             --python_out=${message_binary_include_dir}
             -I${message_binary_include_dir}
             -I${CMAKE_CURRENT_SOURCE_DIR}/proto
             "${outputpath}/${file_we}.proto"
        DEPENDS ${binary_depends}
        COMMENT "Compiling protocol buffer ${proto}")

    # Build our c++ class from the extracted information
    ADD_CUSTOM_COMMAND(
        OUTPUT "${outputpath}/${file_we}.cpp"
               "${outputpath}/${file_we}.py.cpp"
               "${outputpath}/${file_we}.h"
        COMMAND ${PYTHON_EXECUTABLE}
        ARGS "${CMAKE_CURRENT_SOURCE_DIR}/build_message_class.py" "${outputpath}/${file_we}" ${outputpath}
        WORKING_DIRECTORY ${message_binary_dir}
        DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/build_message_class.py"
                ${message_class_generator_depends}
                "${message_binary_include_dir}/Neutron_pb2.py"
                "${outputpath}/${file_we}.pb"
        COMMENT "Building classes for ${proto}")

    # The protobuf descriptions are generated
    SET_SOURCE_FILES_PROPERTIES("${outputpath}/${file_we}.pb"
                                "${outputpath}/${file_we}.proto"
                                "${outputpath}/${file_we}.pb.cc"
                                "${outputpath}/${file_we}.pb.h"
                                "${outputpath}/${file_we}.cpp"
                                "${outputpath}/${file_we}.py.cpp"
                                "${outputpath}/${file_we}.h"
                                 PROPERTIES GENERATED TRUE
                                            # Prevent Effective C++ and unused parameter error checks being performed on generated files.
                                            COMPILE_FLAGS "-Wno-effc++ -Wno-error=unused-parameter -Wno-error")

    # Add the generated files to our list
    SET(src ${src}
            "${outputpath}/${file_we}.pb.cc"
            "${outputpath}/${file_we}.pb.h"
            "${outputpath}/${file_we}.cpp"
            "${outputpath}/${file_we}.h")

    # If we have pybind11 also add the python bindings
    IF(pybind11_FOUND)
        SET(src ${src} "${outputpath}/${file_we}.py.cpp")
    ENDIF()

ENDFOREACH(proto)

# If we have pybind11 we need to generate our final binding class
IF(pybind11_FOUND)
    # Build our outer python binding wrapper class
    ADD_CUSTOM_COMMAND(
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/outer_python_binding.cpp"
        COMMAND ${PYTHON_EXECUTABLE}
        ARGS "${CMAKE_CURRENT_SOURCE_DIR}/build_outer_python_binding.py"
             "${CMAKE_CURRENT_BINARY_DIR}/outer_python_binding.cpp"
             "${PROJECT_SOURCE_DIR}/${NUCLEAR_MESSAGE_DIR}"
              ${message_dependencies}
        WORKING_DIRECTORY ${message_binary_dir}
        DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/build_outer_python_binding.py" ${message_dependencies}
        COMMENT "Building outer python message binding")

    SET(src ${src} "${CMAKE_CURRENT_BINARY_DIR}/outer_python_binding.cpp")
ENDIF()

IF(src)
    # Build a library from these files
    ADD_LIBRARY(nuclear_message SHARED ${protobufs} ${src})
    SET_PROPERTY(TARGET nuclear_message PROPERTY LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin/lib")

    # The library uses protocol buffers
    TARGET_LINK_LIBRARIES(nuclear_message ${PROTOBUF_LIBRARIES})
    TARGET_LINK_LIBRARIES(nuclear_message ${NUClear_LIBRARIES})

    # If we have pybind11 we need to make this a python library too
    IF(pybind11_FOUND)
        TARGET_LINK_LIBRARIES(nuclear_message ${PYTHON_LIBRARIES})

        # Work out what python expects the name of the library to be
        SET(python_module_path "${PYTHON_MODULE_PREFIX}message${PYTHON_MODULE_EXTENSION}")

        # Make our NUClear python directory for including
        FILE(MAKE_DIRECTORY "${PROJECT_BINARY_DIR}/python/nuclear")

        # Create symlinks to the files
        ADD_CUSTOM_COMMAND(TARGET nuclear_message POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:nuclear_message> "${PROJECT_BINARY_DIR}/python/nuclear/${python_module_path}"
            COMMENT "Copying messages lib into python file format"
        )

        ADD_CUSTOM_COMMAND(TARGET nuclear_message POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy "${NUCLEAR_ROLES_DIR}/module/python/nuclear.py" "${PROJECT_BINARY_DIR}/python/nuclear/nuclear.py"
            DEPENDS "${NUCLEAR_ROLES_DIR}/module/python/nuclear.py"
            COMMENT "Copying nuclear.py to python build directory"
        )

    ENDIF()

    # Add to our list of NUClear message libraries
    SET(NUCLEAR_MESSAGE_LIBRARIES nuclear_message CACHE INTERNAL "List of libraries that are built as messages" FORCE)

    # Put it in an IDE group for shared
    SET_PROPERTY(TARGET nuclear_message PROPERTY FOLDER "shared/")
ENDIF()
