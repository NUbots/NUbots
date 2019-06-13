# Copyright (C) 2017 Trent Houliston <trent@houliston.me>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
# documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
# Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
# WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
# OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0)
PROJECT(VisualMesh VERSION 1.0.0)

# Check if we are built as a subproject
IF(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
    SET(TOP_LEVEL_PROJECT ON)
ELSE()
    SET(TOP_LEVEL_PROJECT OFF)
ENDIF()

# All the header files for this library
FILE(GLOB hdr "src/**.hpp")

# Convert our opencl files into header files with their contents and put them in the build folder
FILE(GLOB cls "src/engine/opencl/kernels/**.cl")
FOREACH(cl IN LISTS cls)

    FILE(RELATIVE_PATH cl_path ${CMAKE_CURRENT_SOURCE_DIR} ${cl})

    ADD_CUSTOM_COMMAND(
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${cl_path}.hpp"
        COMMAND ${PYTHON_EXECUTABLE}
        ARGS "${PROJECT_SOURCE_DIR}/cmake/Scripts/wrap_opencl.py"
            "${cl}"
            "${CMAKE_CURRENT_BINARY_DIR}/${cl_path}.hpp"
        DEPENDS "${cl}"
                "${PROJECT_SOURCE_DIR}/cmake/Scripts/wrap_opencl.py"
        COMMENT "Wrapping OpenCL file ${cl_path} in a header"
    )
    LIST(APPEND hdr "${CMAKE_CURRENT_BINARY_DIR}/${cl_path}.hpp")
ENDFOREACH(cl ${cls})

ADD_CUSTOM_TARGET(visualmesh_sources DEPENDS ${hdr})

# Define the VisualMesh library as header only but setup it's links
ADD_LIBRARY(libvisualmesh INTERFACE)
TARGET_INCLUDE_DIRECTORIES(libvisualmesh INTERFACE src)
TARGET_INCLUDE_DIRECTORIES(libvisualmesh INTERFACE ${CMAKE_CURRENT_BINARY_DIR}/src)
ADD_DEPENDENCIES(libvisualmesh visualmesh_sources)

# Grab OpenCL if it's available
FIND_PACKAGE(OpenCL)
IF(OpenCL_FOUND)
    TARGET_INCLUDE_DIRECTORIES(libvisualmesh INTERFACE ${OpenCL_INCLUDE_DIRS})
    TARGET_LINK_LIBRARIES(libvisualmesh INTERFACE ${OpenCL_LIBRARIES})
ENDIF(OpenCL_FOUND)

IF(TOP_LEVEL_PROJECT)
    # Build the tensorflow op for training
    OPTION(BUILD_TENSORFLOW_OP "Build the tensorflow op used for training" ON)
    IF(BUILD_TENSORFLOW_OP)
        # Add this directory so we can find the FindTensorflow.cmake
        SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")
        FIND_PACKAGE(TensorFlow REQUIRED)

        ADD_LIBRARY(tf_op SHARED "src/tf_op.cpp" ${hdr})
        TARGET_COMPILE_OPTIONS(tf_op PRIVATE -march=native -mtune=native)
        SET_TARGET_PROPERTIES(tf_op PROPERTIES
            CXX_STANDARD 14
            LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/training
            PREFIX ""
            OUTPUT_NAME visualmesh_op
            SUFFIX ".so"
        )
        TARGET_INCLUDE_DIRECTORIES(tf_op SYSTEM PRIVATE ${TENSORFLOW_INCLUDE_DIRS})
        TARGET_INCLUDE_DIRECTORIES(tf_op PRIVATE ${PROJECT_SOURCE_DIR}/src)
        TARGET_LINK_LIBRARIES(tf_op ${TENSORFLOW_LIBRARIES})

    ENDIF(BUILD_TENSORFLOW_OP)

    # Build an example program that classifies using the visual mesh
    OPTION(BUILD_EXAMPLE "Build the example program that executes the mesh" OFF)
    IF(BUILD_EXAMPLE)

        FIND_PACKAGE(OpenCV REQUIRED)
        FIND_PACKAGE(YAML-CPP REQUIRED)
        FIND_PACKAGE(OpenCL REQUIRED)

        ADD_EXECUTABLE(main "example/main.cpp" ${hdr})
        TARGET_COMPILE_OPTIONS(main PRIVATE -march=native -mtune=native)
        SET_TARGET_PROPERTIES(main PROPERTIES CXX_STANDARD 14)
        TARGET_INCLUDE_DIRECTORIES(main SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
        TARGET_INCLUDE_DIRECTORIES(main SYSTEM PRIVATE ${YAML_CPP_INCLUDE_DIR})
        TARGET_INCLUDE_DIRECTORIES(main PRIVATE ${PROJECT_SOURCE_DIR}/src)
        TARGET_INCLUDE_DIRECTORIES(main PRIVATE ${PROJECT_BINARY_DIR}/src)
        TARGET_LINK_LIBRARIES(main ${OpenCL_LIBRARIES} ${OpenCV_LIBS} ${fmt_LIBRARIES} ${YAML_CPP_LIBRARIES})
    ENDIF(BUILD_EXAMPLE)
ENDIF(TOP_LEVEL_PROJECT)