/*
 * Copyright (C) 2017 Trent Houliston <trent@houliston.me>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef VISUALMESH_HPP
#define VISUALMESH_HPP

#include <fmt/format.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <map>
#include <numeric>
#include <sstream>
#include <type_traits>
#include <vector>

#if defined(__APPLE__) || defined(__MACOSX)
#include <OpenCL/opencl.h>
#else
#include <CL/opencl.h>
#endif  // !__APPLE__

// Include our generated OpenCL headers
#include "cl/project_equidistant.cl.hpp"
#include "cl/project_equisolid.cl.hpp"
#include "cl/project_rectilinear.cl.hpp"
#include "cl/read_image_to_network.cl.hpp"
#include "opencl_error_category.hpp"

namespace mesh {

namespace cl {
    template <typename T>
    struct opencl_wrapper : public std::shared_ptr<std::remove_reference_t<decltype(*std::declval<T>())>> {
        using std::shared_ptr<std::remove_reference_t<decltype(*std::declval<T>())>>::shared_ptr;

        T* operator&() {
            ptr = this->get();
            return &ptr;
        }

        operator T() const {
            return this->get();
        }

        size_t size() const {
            return sizeof(T);
        }

    private:
        T ptr = nullptr;
    };

    using command_queue = opencl_wrapper<::cl_command_queue>;
    using context       = opencl_wrapper<::cl_context>;
    using event         = opencl_wrapper<::cl_event>;
    using kernel        = opencl_wrapper<::cl_kernel>;
    using mem           = opencl_wrapper<::cl_mem>;
    using program       = opencl_wrapper<::cl_program>;
}  // namespace cl

template <typename T>
struct LazyBufferReader {

    LazyBufferReader() = default;

    LazyBufferReader(const cl::command_queue& queue,
                     const cl::mem& buffer,
                     const std::vector<cl::event>& ready,
                     uint n_elements)
        : queue(queue), buffer(buffer), ready(ready), n_elements(n_elements) {}

    LazyBufferReader(const cl::command_queue& queue, const cl::mem& buffer, const cl::event& ready, uint n_elements)
        : queue(queue), buffer(buffer), ready(std::vector<cl::event>({ready})), n_elements(n_elements) {}

    template <typename U>
    std::vector<U> as() const {
        // Number of output elements will change if the sizes are different
        std::vector<U> output(n_elements * sizeof(T) / sizeof(U));

        std::vector<cl_event> events(ready.begin(), ready.end());
        cl_int error = ::clEnqueueReadBuffer(
            queue, buffer, true, 0, n_elements * sizeof(T), output.data(), events.size(), events.data(), nullptr);
        if (error != CL_SUCCESS) {
            throw std::system_error(error, opencl_error_category(), "Error reading vector buffer for lazy evaluation");
        }
        return output;
    }

    operator std::vector<T>() const {
        std::vector<T> output(n_elements);
        std::vector<cl_event> events(ready.begin(), ready.end());
        cl_int error = ::clEnqueueReadBuffer(
            queue, buffer, true, 0, n_elements * sizeof(T), output.data(), events.size(), events.data(), nullptr);
        if (error != CL_SUCCESS) {
            throw std::system_error(error, opencl_error_category(), "Error reading vector buffer for lazy evaluation");
        }
        return output;
    }

    operator T() const {
        T output;
        std::vector<cl_event> events(ready.begin(), ready.end());
        cl_int error =
            ::clEnqueueReadBuffer(queue, buffer, true, 0, sizeof(T), &output, events.size(), events.data(), nullptr);
        if (error != CL_SUCCESS) {
            throw std::system_error(error, opencl_error_category(), "Error reading buffer for lazy evaluation");
        }
        return output;
    }

    cl::command_queue queue;
    cl::mem buffer;
    std::vector<cl::event> ready;
    uint n_elements;
};

/**
 * @brief Constructs and holds a visual mesh
 * @details [long description]
 *
 * @tparam Scalar the type that will hold the vectors <float, double>
 */
template <typename Scalar = float>
class VisualMesh {
public:
    // Typedef some value types we commonly use
    using vec2 = std::array<Scalar, 2>;
    using vec3 = std::array<Scalar, 3>;
    using vec4 = std::array<Scalar, 4>;
    using mat3 = std::array<vec3, 3>;
    using mat4 = std::array<vec4, 4>;

    struct Lens {
        enum Projection { RECTILINEAR, EQUISOLID, EQUIDISTANT };

        Projection projection;
        std::array<int, 2> dimensions;
        Scalar fov;
        Scalar focal_length;
    };

    struct ProjectedMesh {

        // Host side buffers for the data
        LazyBufferReader<std::array<Scalar, 2>> pixel_coordinates;
        std::vector<std::array<int, 6>> neighbourhood;
        std::vector<int> global_indices;

        // OpenCL buffers for the data
        cl::mem cl_pixel_coordinates;
        cl::event cl_pixel_coordinates_event;
        cl::mem cl_neighbourhood;
        cl::event cl_neighbourhood_event;
    };

    struct ClassifiedMesh {

        LazyBufferReader<std::array<Scalar, 2>> pixel_coordinates;
        std::vector<std::array<int, 6>> neighbourhood;
        std::vector<int> global_indices;
        std::vector<std::pair<int, LazyBufferReader<Scalar>>> classifications;
    };

    struct Node {
        /// The unit vector in the direction for this node
        vec4 ray;
        /// Relative indices to the linked hexagon nodes in the LUT ordered TL, TR, L, R, BL, BR,
        std::array<int, 6> neighbours;
    };

    struct Row {
        Row(const Scalar& phi, const int& begin, const int& end) : phi(phi), begin(begin), end(end) {}

        /// The phi value this row represents
        Scalar phi;
        /// The index of the beginning of this row in the node table
        int begin;
        /// The index of one past the end of this row in the node table
        int end;

        /**
         * @brief Compare based on phi
         *
         * @param other other item to compare to
         *
         * @return if our phi is less than other phi
         */
        bool operator<(const Row& other) const {
            return phi < other.phi;
        }
    };

    struct Mesh {
        Mesh(const std::vector<Node>& nodes, const std::vector<Row>& rows, const cl::mem& cl_points)
            : nodes(nodes), rows(rows), cl_points(cl_points) {}

        /// The lookup table for this mesh
        std::vector<Node> nodes;
        /// A set of individual rows for phi values. `begin` and `end` refer to the table with end being 1 past the
        /// end
        std::vector<Row> rows;

        /// The on device buffer of the visual mesh unit vectors
        cl::mem cl_points;
    };

    enum FOURCC : cl_int {
        GREY    = 0x59455247,
        Y12     = 0x20323159,
        Y16     = 0x20363159,
        GRBG    = 0x47425247,
        RGGB    = 0x42474752,
        GBRG    = 0x47524247,
        BGGR    = 0x52474742,
        GR12    = 0x32315247,
        RG12    = 0x32314752,
        GB12    = 0x32314247,
        BG12    = 0x32314742,
        GR16    = 0x36315247,
        RG16    = 0x36314752,
        GB16    = 0x36314247,
        BG16    = 0x36314742,
        Y411    = 0x31313459,
        UYVY    = 0x59565955,
        YUYV    = 0x56595559,
        YM24    = 0x34324d59,
        RGB3    = 0x33424752,
        RGBA    = 0x41424752,
        BGR3    = 0x33524742,
        BGRA    = 0x41524742,
        JPEG    = 0x4745504a,
        UNKNOWN = 0
    };


    class Classifier {
    private:
        using weights_t = std::vector<std::vector<Scalar>>;
        using biases_t  = std::vector<Scalar>;

        using layer_t             = std::pair<weights_t, biases_t>;
        using conv_layer_t        = std::vector<layer_t>;
        using network_structure_t = std::vector<conv_layer_t>;

    public:
        Classifier() : mesh(nullptr) {}

        Classifier(VisualMesh* mesh, const network_structure_t& structure)
            : mesh(mesh), conv_mutex(std::make_shared<std::mutex>()) {

            // Build using a string stream
            std::stringstream code;

            // Set our precision for how many digits a float has
            code << std::setprecision(std::numeric_limits<Scalar>::digits10 + 1);

            auto vector_type = [](const int& size) {
                return (size == 1 || size == 2 || size == 4 || size == 8 || size == 16) ? size : 0;
            };

            for (uint conv_no = 0; conv_no < structure.size(); ++conv_no) {
                auto& conv = structure[conv_no];

                // We need to work out the input and output sizes for our convolution
                int conv_in_size;
                int conv_out_size;

                // On the first convolution we assume an input size of 4
                if (conv_no == 0) {
                    conv_in_size = 4;
                }
                else {
                    // The output dimension of our previous bias vector
                    conv_in_size = structure[conv_no - 1].back().second.size();
                }

                // The output dimension of our last bias vector
                conv_out_size = conv.back().second.size();

                // Work out our input and output types
                std::string in_type("float");
                if (vector_type(conv_in_size)) {
                    in_type.append(std::to_string(conv_in_size));
                }
                std::string out_type("float");
                if (vector_type(conv_out_size)) {
                    out_type.append(std::to_string(conv_out_size));
                }

                // Write our OpenCL kernel definition
                code << "kernel void conv" << conv_no << "(global const int* neighbourhood, global const " << in_type
                     << "* input, global " << out_type << "* output) {" << std::endl
                     << std::endl;

                code << "    // Get our kernel index" << std::endl;
                code << "    const int idx = get_global_id(0);" << std::endl << std::endl;

                /*************************************************
                 *                    GATHER                     *
                 *************************************************/

                code << "    // Gather from our neighbourhood " << std::endl;
                if (vector_type(conv_in_size)) {
                    code << "    " << in_type << " in0[7] = {" << std::endl;
                    code << "        input[idx]," << std::endl;
                    for (int i = 0; i < 6; ++i) {
                        code << "        input[neighbourhood[idx * 6 + " << i << "]]";
                        if (i != 5) {
                            code << ",";
                        }
                        code << std::endl;
                    }
                    code << "    };";
                }
                // Perform our gather step for non vectorized data
                else {
                    code << "    float in0[" << (conv_in_size * 7) << "] = {" << std::endl;

                    // Read the ones for our own index
                    for (int j = 0; j < conv_in_size; ++j) {
                        code << "        input[idx * " << conv_in_size << " + " << j << "]";
                    }

                    // Read our neighbourhood
                    for (int i = 0; i < 6; ++i) {
                        for (int j = 0; j < conv_in_size; ++j) {
                            code << "        input[neighbourhood[idx * 6 + " << i << "] * " << conv_in_size << " + "
                                 << j << "]";

                            if (i < 6 || j + 1 < conv_in_size) {
                                code << ",";
                            }
                            code << std::endl;
                        }
                    }
                    code << "    };";
                }

                code << std::endl << std::endl;

                /*************************************************
                 *                WEIGHTS + BIAS                 *
                 *************************************************/

                // Now we have to do our layer operations
                int in_size = conv_in_size;
                for (uint layer_no = 0; layer_no < conv.size(); ++layer_no) {
                    const auto& weights = conv[layer_no].first;
                    const auto& biases  = conv[layer_no].second;

                    const int vector_in  = vector_type(in_size);
                    const int vector_out = vector_type(biases.size());

                    code << "    // Perform our matrix multiplication for weights and add bias for layer " << layer_no
                         << std::endl;

                    // Open our next input (either vector or not)
                    if (vector_out) {
                        code << "    float" << vector_out << " in" << (layer_no + 1) << " = (float" << vector_out
                             << ")(" << std::endl;
                    }
                    else {
                        code << "    float in" << (layer_no + 1) << "[" << biases.size() << "] = {" << std::endl;
                    }

                    // Matrix multiplication + bias
                    if (vector_in) {
                        for (uint i = 0; i < biases.size(); ++i) {
                            code << "        ";
                            for (uint j = 0; j < weights.size(); j += vector_in) {

                                // If our data is gathered, we need to get our gathered index
                                std::string gathered_index =
                                    layer_no == 0 ? "[" + std::to_string(j / vector_in) + "]" : "";

                                // Dot our element with our fixed data
                                code << "dot(in" << layer_no << gathered_index << ", (float" << vector_in << ")(";

                                // Write our fixed data
                                for (uint k = j; k < j + vector_in; ++k) {
                                    code << weights[k][i];
                                    if (k + 1 < j + vector_in) {
                                        code << ", ";
                                    }
                                }

                                // End
                                code << ")) + ";
                            }
                            code << biases[i];
                            if (i + 1 < biases.size()) {
                                code << ",";
                            }
                            code << std::endl;
                        }
                    }
                    else {
                        for (uint i = 0; i < biases.size(); ++i) {
                            code << "        ";
                            for (uint j = 0; j < weights.size(); ++j) {
                                code << "in" << layer_no << "[" << j << "] * " << weights[j][i] << " + ";
                            }
                            code << biases[i];
                            if (i + 1 < biases.size()) {
                                code << ",";
                            }
                            code << std::endl;
                        }
                    }

                    // Close our output
                    if (vector_out) {
                        code << "    );";
                    }
                    else {
                        code << "    };";
                    }
                    code << std::endl << std::endl;


                    /*************************************************
                     *                  ACTIVATION.                  *
                     *************************************************/

                    // Apply our activation function
                    code << "    // Apply the activation function" << std::endl;

                    // selu constants
                    constexpr const float lambda = 1.0507009873554804934193349852946;
                    constexpr const float alpha  = 1.6732632423543772848170429916717;

                    // Apply selu
                    if (vector_out) {
                        std::string e = "in" + std::to_string(layer_no + 1);

                        code << "    " << e << " = " << lambda << "f * select(" << alpha << "f * exp(" << e << ") - "
                             << alpha << "f, in" << (layer_no + 1) << ", " << e << " > 0);"
                             << std::endl;  // select(a, b, c) == c ? b : a
                    }
                    else {
                        for (uint i = 0; i < biases.size(); ++i) {
                            std::string e = "in" + std::to_string(layer_no + 1) + "[" + std::to_string(i) + "]";
                            code << "    " << e << " = " << lambda << "f * (" << e << " > 0 ? " << e << " : " << alpha
                                 << "f * exp(" << e << ") - " << alpha << "f);" << std::endl;
                        }
                    }
                    code << std::endl;

                    // If this is our last layer, apply softmax
                    if (conv_no + 1 == structure.size() && layer_no + 1 == conv.size()) {
                        code << "    // Apply softmax to our final output" << std::endl;

                        if (vector_out) {
                            std::string e = "in" + std::to_string(layer_no + 1);
                            code << "    " << e << " = exp(" << e << ");" << std::endl;
                            code << "    " << e << " = " << e << " / dot(" << e << ", (float" << vector_out << ")(1));"
                                 << std::endl;
                        }
                        else {

                            // Apply exp to each of the elements
                            for (uint i = 0; i < biases.size(); ++i) {
                                std::string e = "in" + std::to_string(layer_no + 1) + "[" + std::to_string(i) + "]";
                                code << "    " << e << " = exp(" << e << ");" << std::endl;
                            }

                            // Sum up all the values
                            code << "float exp_sum = 0";
                            for (uint i = 0; i < biases.size(); ++i) {
                                std::string e = "in" + std::to_string(layer_no + 1) + "[" + std::to_string(i) + "]";
                                code << "    exp_sum += " << e << ";" << std::endl;
                            }

                            // Divide all the values
                            for (uint i = 0; i < biases.size(); ++i) {
                                std::string e = "in" + std::to_string(layer_no + 1) + "[" + std::to_string(i) + "]";
                                code << "    " << e << " /= exp_sum" << std::endl;
                            }
                        }

                        code << std::endl;
                    }

                    // Update our input size for the next loop
                    in_size = biases.size();
                }

                /*************************************************
                 *                    OUTPUT                     *
                 *************************************************/
                code << "    // Save our value to the output" << std::endl;
                if (vector_type(conv_out_size)) {
                    code << "    output[idx] = "
                         << "in" << conv.size() << ";" << std::endl;
                }
                else {
                    for (int i = 0; i < conv_out_size; ++i) {
                        code << "    output[idx * " << conv_out_size << " + " << i << "] = in" << conv.size() << "["
                             << i << "];" << std::endl;
                    }
                }

                code << "}" << std::endl << std::endl;
            }

            // Create our OpenCL program, compile it and get our kernels
            cl_int error;
            std::string source = code.str();
            const char* cstr   = source.c_str();
            size_t csize       = source.size();

            program =
                cl::program(::clCreateProgramWithSource(mesh->context, 1, &cstr, &csize, &error), ::clReleaseProgram);

            if (error != CL_SUCCESS) {
                throw std::system_error(error, opencl_error_category(), "Error adding sources to classifier program");
            }

            // Compile the program
            error = ::clBuildProgram(
                program, 0, nullptr, "-cl-single-precision-constant -cl-fast-relaxed-math", nullptr, nullptr);
            if (error != CL_SUCCESS) {

                // Get the first device
                cl_device_id device;
                ::clGetContextInfo(mesh->context, CL_CONTEXT_DEVICES, sizeof(cl_device_id), &device, nullptr);

                // Get program build log
                size_t used = 0;
                ::clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG, 0, nullptr, &used);
                std::vector<char> log(used);
                ::clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG, log.size(), log.data(), &used);

                // Throw an error with the build log
                throw std::system_error(
                    error,
                    opencl_error_category(),
                    "Error building classifier program\n" + std::string(log.begin(), log.begin() + used));
            }

            for (uint i = 0; i < structure.size(); ++i) {
                std::string kernel = "conv" + std::to_string(i);
                uint output_size   = structure[i].back().second.size();

                cl_int error;
                cl::kernel k(::clCreateKernel(program, kernel.c_str(), &error), ::clReleaseKernel);
                if (error != CL_SUCCESS) {
                    throw std::system_error(error, opencl_error_category(), "Failed to create kernel " + kernel);
                }
                else {
                    conv_layers.emplace_back(k, output_size);
                }
            }
        }

        ClassifiedMesh operator()(const void* image, const FOURCC& format, const mat4& Hoc, const Lens& lens) {


            cl_image_format fmt;

            switch (format) {
                // Bayer
                case GRBG:
                case RGGB:
                case GBRG:
                case BGGR: fmt = cl_image_format{CL_R, CL_UNORM_INT8}; break;
                case BGRA: fmt = cl_image_format{CL_BGRA, CL_UNORM_INT8}; break;
                case RGBA: fmt = cl_image_format{CL_RGBA, CL_UNORM_INT8}; break;
                // Oh no...
                default: throw std::runtime_error("Unsupported image format");
            }

            cl_image_desc desc = {CL_MEM_OBJECT_IMAGE2D,
                                  size_t(lens.dimensions[0]),
                                  size_t(lens.dimensions[1]),
                                  1,
                                  1,
                                  0,
                                  0,
                                  0,
                                  0,
                                  nullptr};

            // Create a buffer for our image
            cl_int error;
            cl::mem img(::clCreateImage(mesh->context,
                                        CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR,
                                        &fmt,
                                        &desc,
                                        const_cast<void*>(image),
                                        &error),
                        ::clReleaseMemObject);
            if (error != CL_SUCCESS) {
                throw std::system_error(error, opencl_error_category(), "Error creating image on device");
            }

            // Map our image into device memory
            std::array<size_t, 3> origin = {{0, 0, 0}};
            std::array<size_t, 3> region = {{size_t(lens.dimensions[0]), size_t(lens.dimensions[1]), 1}};

            cl::event img_event;
            cl_event ev           = nullptr;
            std::size_t row_pitch = 0;
            ::clEnqueueMapImage(mesh->queue,
                                img,
                                false,
                                CL_MAP_READ,
                                origin.data(),
                                region.data(),
                                &row_pitch,
                                nullptr,
                                0,
                                nullptr,
                                &ev,
                                &error);
            if (ev) img_event = cl::event(ev, ::clReleaseEvent);
            if (error != CL_SUCCESS) {
                throw std::system_error(error, opencl_error_category(), "Error mapping image onto device");
            }

            // Project our visual mesh
            auto projection = mesh->project(Hoc, lens);

            // This includes the offscreen point at the end
            int points = projection.neighbourhood.size();


            // First layer, output from the image
            cl::mem img_load_buffer(
                ::clCreateBuffer(mesh->context, CL_MEM_READ_WRITE, sizeof(cl_float4) * points, nullptr, &error),
                ::clReleaseMemObject);
            if (error != CL_SUCCESS) {
                throw std::system_error(error, opencl_error_category(), "Error allocating buffer on device");
            }

            // Zero out the final value in the buffer
            cl::event offscreen_fill_event;
            ev            = nullptr;
            float minus_1 = -1.0f;
            error         = ::clEnqueueFillBuffer(mesh->queue,
                                          img_load_buffer,
                                          &minus_1,
                                          sizeof(float),
                                          (points - 1) * sizeof(cl_float4),
                                          sizeof(cl_float4),
                                          0,
                                          nullptr,
                                          &ev);
            if (ev) offscreen_fill_event = cl::event(ev, ::clReleaseEvent);
            if (error != CL_SUCCESS) {
                throw std::system_error(error, opencl_error_category(), "Error setting the offscreen pixel values");
            }

            // Read the pixels into the buffer
            cl::event img_load_event;
            ev = nullptr;
            /* Mutex scope */ {
                std::lock_guard<std::mutex> lock(mesh->read_image_to_network_mutex);

                error = ::clSetKernelArg(mesh->read_image_to_network, 0, img.size(), &img);
                if (error != CL_SUCCESS) {
                    throw std::system_error(
                        error, opencl_error_category(), "Error setting kernel argument 0 for image load kernel");
                }
                error = ::clSetKernelArg(mesh->read_image_to_network, 1, sizeof(format), &format);
                if (error != CL_SUCCESS) {
                    throw std::system_error(
                        error, opencl_error_category(), "Error setting kernel argument 1 for image load kernel");
                }
                error = ::clSetKernelArg(mesh->read_image_to_network,
                                         2,
                                         projection.cl_pixel_coordinates.size(),
                                         &projection.cl_pixel_coordinates);
                if (error != CL_SUCCESS) {
                    throw std::system_error(
                        error, opencl_error_category(), "Error setting kernel argument 2 for image load kernel");
                }
                error = ::clSetKernelArg(mesh->read_image_to_network, 3, img_load_buffer.size(), &img_load_buffer);
                if (error != CL_SUCCESS) {
                    throw std::system_error(
                        error, opencl_error_category(), "Error setting kernel argument 3 for image load kernel");
                }

                size_t offset[1]       = {0};
                size_t global_size[1]  = {size_t(points - 1)};
                cl_event event_list[2] = {projection.cl_pixel_coordinates_event, img_event};
                error                  = ::clEnqueueNDRangeKernel(mesh->queue,
                                                 mesh->read_image_to_network,
                                                 1,
                                                 offset,
                                                 global_size,  // -1 as we don't project the offscreen point
                                                 nullptr,
                                                 2,
                                                 event_list,
                                                 &ev);
                if (ev) img_load_event = cl::event(ev, ::clReleaseEvent);
                if (error != CL_SUCCESS) {
                    throw std::system_error(error, opencl_error_category(), "Error queueing the image load kernel");
                }
            }

            // Our buffers for each layer
            std::vector<std::pair<cl::mem, std::vector<cl::event>>> layer_buffers;

            // These make up our first buffers
            layer_buffers.emplace_back(
                img_load_buffer,
                std::vector<cl::event>({img_load_event, offscreen_fill_event, projection.cl_neighbourhood_event}));

            // Run each of our conv layers
            /* Mutex Scope */ {
                std::lock_guard<std::mutex> lock(*conv_mutex);

                for (auto& conv : conv_layers) {

                    // Create an output buffer
                    cl::mem out_buffer(::clCreateBuffer(mesh->context,
                                                        CL_MEM_READ_WRITE,
                                                        size_t(conv.second * points * sizeof(float)),
                                                        nullptr,
                                                        &error),
                                       ::clReleaseMemObject);
                    if (error) {
                        throw std::system_error(
                            error, opencl_error_category(), "Error creating output buffer for the convolution kernel");
                    }

                    error = ::clSetKernelArg(
                        conv.first, 0, projection.cl_neighbourhood.size(), &projection.cl_neighbourhood);
                    if (error) {
                        throw std::system_error(
                            error, opencl_error_category(), "Error setting argument 0 for convolution kernel");
                    }
                    error =
                        ::clSetKernelArg(conv.first, 1, layer_buffers.back().first.size(), &layer_buffers.back().first);
                    if (error) {
                        throw std::system_error(
                            error, opencl_error_category(), "Error setting argument 1 for convolution kernel");
                    }
                    error = ::clSetKernelArg(conv.first, 2, out_buffer.size(), &out_buffer);
                    if (error) {
                        throw std::system_error(
                            error, opencl_error_category(), "Error setting argument 2 for convolution kernel");
                    }


                    // Convert our events into
                    std::vector<cl_event> events(layer_buffers.back().second.begin(),
                                                 layer_buffers.back().second.end());

                    size_t offset[1]      = {0};
                    size_t global_size[1] = {size_t(points)};
                    cl::event event;
                    ev    = nullptr;
                    error = ::clEnqueueNDRangeKernel(
                        mesh->queue, conv.first, 1, offset, global_size, nullptr, events.size(), events.data(), &ev);
                    if (ev) event = cl::event(ev, ::clReleaseEvent);
                    if (error) {
                        throw std::system_error(error, opencl_error_category(), "Error queueing convolution kernel");
                    }

                    layer_buffers.emplace_back(out_buffer, std::vector<cl::event>({event}));
                }
            }

            // Flush the queue to ensure it has executed
            ::clFlush(mesh->queue);

            std::vector<std::pair<int, LazyBufferReader<Scalar>>> outputs;
            for (uint i = 0; i < layer_buffers.size(); ++i) {

                uint dims = i == 0 ? 4 : conv_layers[i - 1].second;

                outputs.emplace_back(dims,
                                     LazyBufferReader<Scalar>(
                                         mesh->queue, layer_buffers[i].first, layer_buffers[i].second, points * dims));
            }

            return ClassifiedMesh{projection.pixel_coordinates,
                                  std::move(projection.neighbourhood),
                                  std::move(projection.global_indices),
                                  std::move(outputs)};
        }

    private:
        VisualMesh* mesh;
        cl::program program;
        std::vector<std::pair<cl::kernel, int>> conv_layers;
        std::shared_ptr<std::mutex> conv_mutex;
    };

    /**
     * @brief Makes an unallocated visual mesh
     */
    VisualMesh() {}

    /**
     * @brief Generate a new visual mesh for the given shape.
     *
     * @param shape             the shape we are generating a visual mesh for
     * @param min_height        the minimum height that our camera will be at
     * @param max_height        the maximum height our camera will be at
     * @param height_resolution the number of look up tables to generated (height gradations)
     * @param min_angular_res   the smallest angular size to generate for
     */
    template <typename Shape>
    explicit VisualMesh(const Shape& shape,
                        const Scalar& min_height,
                        const Scalar& max_height,
                        const uint& height_resolution,
                        const Scalar& min_angular_res)
        : min_angular_res(min_angular_res)
        , min_height(min_height)
        , max_height(max_height)
        , height_resolution(height_resolution) {

        // Setup OpenCL
        setup_opencl();

        // Loop through to make a mesh for each of our height possibilities
        for (Scalar h = min_height; h < max_height; h += (max_height - min_height) / height_resolution) {

            // This is a list of phi values along with the delta theta values associated with them
            std::vector<std::pair<Scalar, int>> phis;

            // Add our 0 point at the bottom if that is a valid location
            if (shape.phi(Scalar(0.0), h) < Scalar(M_PI_2)) {
                phis.emplace_back(Scalar(0.0), 1);
            }

            // Loop from directly down up to the horizon (if phi is nan it will stop)
            // So we don't have a single point at the base, we move half a jump forward
            for (Scalar phi = shape.phi(Scalar(0.0), h); phi < Scalar(M_PI_2);) {

                // Calculate our theta
                Scalar theta = std::max(shape.theta(phi, h), min_angular_res);

                if (!std::isnan(theta)) {
                    // Push back the phi, and the number of whole shapes we can fit
                    phis.emplace_back(phi, uint(std::ceil(Scalar(2.0) * M_PI / theta)));
                }

                // Calculate our next phi value
                const Scalar new_phi = shape.phi(phi, h);

                // Apply our min jump if the new values is not nan
                phi = std::isnan(new_phi) ? new_phi : std::max(phi + min_angular_res, new_phi);
            }

            // Add our 0 point at the bottom if that is a valid location
            if (Scalar(M_PI) + shape.phi(M_PI, h) > Scalar(M_PI_2)) {
                phis.emplace_back(Scalar(M_PI), 1);
            }

            // Loop from directly up down to the horizon (if phi is nan it will stop)
            for (Scalar phi = Scalar(M_PI) + shape.phi(M_PI, h); phi > Scalar(M_PI_2);) {

                // Calculate our theta
                Scalar theta = std::max(shape.theta(phi, h), min_angular_res);

                if (!std::isnan(theta)) {
                    // Push back the phi, and the number of whole shapes we can fit
                    phis.emplace_back(phi, uint(std::ceil(Scalar(2.0) * M_PI / theta)));
                }

                // Calculate our next phi value
                const Scalar new_phi = shape.phi(phi, h);

                // Move to our next phi
                phi = std::isnan(new_phi) ? new_phi : std::min(phi - min_angular_res, new_phi);
            }


            // Sort the list by phi to create a contiguous area
            std::sort(phis.begin(), phis.end());

            // From this generate unit vectors for the full lut
            std::vector<Node> lut;

            // Work out how big our LUT will be
            uint lut_size = 0;
            for (const auto& v : phis) {
                lut_size += v.second;
            }
            lut.reserve(lut_size);

            // The start and end of each row in the final lut
            std::vector<Row> rows;
            rows.reserve(phis.size());

            // Loop through our LUT and calculate our left and right neighbours
            for (const auto& v : phis) {

                // Get our phi and delta theta values for a clean circle
                const auto& phi      = v.first;
                const Scalar sin_phi = std::sin(phi);
                const Scalar cos_phi = std::cos(phi);
                const auto& steps    = v.second;
                const Scalar dtheta  = (Scalar(2.0) * M_PI) / steps;

                // We will use the start position of each row later for linking the graph
                rows.emplace_back(phi, lut.size(), lut.size() + steps);

                // Generate for each of the theta values from 0 to 2 pi
                Scalar theta = 0;
                for (int i = 0; i < steps; ++i) {
                    Node n;

                    // Calculate our unit vector with origin facing forward
                    n.ray = {{
                        std::cos(theta) * sin_phi,  //
                        std::sin(theta) * sin_phi,  //
                        -cos_phi,                   //
                        Scalar(0.0)                 //
                    }};

                    // Get the indices for our left/right neighbours relative to this row
                    const int l = i == 0 ? steps - 1 : i - 1;
                    const int r = i == steps - 1 ? 0 : i + 1;

                    // Set these two neighbours and default the others to ourself
                    n.neighbours[0] = 0;
                    n.neighbours[1] = 0;
                    n.neighbours[2] = l - i;  // L
                    n.neighbours[3] = r - i;  // R
                    n.neighbours[4] = 0;
                    n.neighbours[5] = 0;

                    // Move on to the next theta value
                    theta += dtheta;

                    lut.push_back(std::move(n));
                }
            }


            /**
             * This function links to the previous and next rows
             *
             * @param i       the absolute index to the node we are linking
             * @param pos     the position of this node in its row as a value between 0 and 1
             * @param start   the start of the row to link to
             * @param size    the size of the row we are linking to
             * @param offset  the offset for our neighbour (0 for TL,TR 4 for BL BR)
             */
            auto link = [](std::vector<Node>& lut,
                           const int& i,
                           const Scalar& pos,
                           const int& start,
                           const int& size,
                           const uint offset) {
                // Grab our current node
                auto& node = lut[i];

                // If the size of the row we are linking to is 1, all elements will link to it
                // This is the case for the very first and very last row
                if (size == 1) {

                    // Now use these to set our TL and TR neighbours
                    node.neighbours[offset]     = start - i;
                    node.neighbours[offset + 1] = start - i;
                }
                else {
                    // Work out if we are closer to the left or right and make an offset var for it
                    // Note this bool is used like a bool and int. It is 0 when we should access TR first
                    // and 1 when we should access TL first. This is to avoid accessing values which wrap around
                    // and instead access a non wrap element and use its neighbours to work out ours
                    const bool left = pos > Scalar(0.5);

                    // Get our closest neighbour on the previous row and use it to work out where the other one
                    // is This will be the Right element when < 0.5 and Left when > 0.5
                    const int o1 = start + std::floor(pos * size + !left);  // Use `left` to add one to one
                    const int o2 = o1 + lut[o1].neighbours[2 + left];       // But not the other

                    // Now use these to set our TL and TR neighbours
                    node.neighbours[offset]     = (left ? o1 : o2) - i;
                    node.neighbours[offset + 1] = (left ? o2 : o1) - i;
                }
            };

            // Now we upwards and downwards to fill in the missing links
            for (uint r = 1; r + 1 < rows.size(); ++r) {

                // Alias for convenience
                const auto& prev    = rows[r - 1];
                const auto& current = rows[r];
                const auto& next    = rows[r + 1];

                // Work out how big our rows are if they are within valid indices
                const int prev_size    = prev.end - prev.begin;
                const int current_size = current.end - current.begin;
                const int next_size    = next.end - next.begin;

                // Go through all the nodes on our current row
                for (int i = current.begin; i < current.end; ++i) {

                    // Find where we are in our row as a value between 0 and 1
                    const Scalar pos = Scalar(i - current.begin) / Scalar(current_size);

                    // Perform both links
                    link(lut, i, pos, prev.begin, prev_size, 0);
                    link(lut, i, pos, next.begin, next_size, 4);
                }
            }

            // Now we have to deal with the very first, and very last rows as they can't be linked in the normal way
            if (!rows.empty()) {
                const auto& front    = rows.front();
                const int front_size = front.end - front.begin;

                const auto& row_2    = rows.size() > 1 ? rows[1] : rows.front();
                const int row_2_size = row_2.end - row_2.begin;

                const auto& back    = rows.back();
                const int back_size = back.end - back.begin;

                const auto& row_2_last    = rows.size() > 1 ? rows[rows.size() - 1] : rows.back();
                const int row_2_last_size = row_2_last.end - row_2_last.begin;

                // Link to our next row in a circle
                if (front_size == 1) {
                    Scalar delta(Scalar(row_2_size) / Scalar(6.0));
                    auto& n = lut.front().neighbours;
                    for (int i = 0; i < 6; ++i) {
                        // Get the position on the next row
                        n[i] = row_2.begin + int(std::round(delta * i));
                    }
                }

                // Link to our next row in a circle
                if (back_size == 1) {
                    Scalar delta(Scalar(row_2_last_size) / Scalar(6.0));
                    auto& n = lut.back().neighbours;
                    for (int i = 0; i < 6; ++i) {
                        // Get the position on the previous row
                        n[i] = row_2_last.begin + int(std::round(delta * i)) - (lut.size() - 1);
                    }
                }
            }

            // Flatten out our memory for opencl
            std::vector<std::array<Scalar, 4>> cl_points;
            cl_points.reserve(lut.size());
            for (const auto& n : lut) {
                cl_points.push_back(n.ray);
            }

            // Convert all the relative indices we calculated to absolute indices
            for (uint i = 0; i < lut.size(); ++i) {
                for (auto& n : lut[i].neighbours) {
                    n = i + n;
                }
            }

            // Upload our unit vectors to the OpenCL device
            cl_int error;
            cl::mem cl_points_buffer(
                ::clCreateBuffer(
                    context, CL_MEM_READ_ONLY, cl_points.size() * sizeof(std::array<Scalar, 4>), nullptr, &error),
                ::clReleaseMemObject);
            if (error) {
                throw std::system_error(error, opencl_error_category(), "Error allocating lookup table buffer");
            }

            error = ::clEnqueueWriteBuffer(queue,
                                           cl_points_buffer,
                                           true,
                                           0,
                                           cl_points.size() * sizeof(std::array<Scalar, 4>),
                                           cl_points.data(),
                                           0,
                                           nullptr,
                                           nullptr);

            // Ensure everything is done
            clFinish(queue);

            if (error) {
                throw std::system_error(error, opencl_error_category(), "Error uploading lookup table to device");
            }

            // Insert our constructed mesh into the lookup
            luts.insert(std::make_pair(h, Mesh(std::move(lut), std::move(rows), cl_points_buffer)));
        }
    }

    const Mesh& height(const Scalar& height) const {
        // Find the bounding height values
        auto range = luts.equal_range(height);

        // If we reached the end of the list return the lower bound
        if (range.second == luts.end()) {
            return range.first->second;
        }
        // Otherwise see which is closer
        else if (std::abs(range.first->first - height) < std::abs(range.second->first - height)) {
            return range.first->second;
        }
        else {
            return range.second->second;
        }
    }

    template <typename Func>
    std::pair<const Mesh&, std::vector<std::pair<uint, uint>>> lookup(const Scalar& height, Func&& theta_limits) const {

        const auto& mesh = this->height(height);
        std::vector<std::pair<uint, uint>> indices;

        // Loop through each phi row
        for (const auto& row : mesh.rows) {

            const auto row_size = row.end - row.begin;

            // Get the theta values that are valid for this phi
            const auto theta_ranges = theta_limits(row.phi);

            // Work out what this range means in terms of theta
            for (const auto& range : theta_ranges) {

                // Convert our theta values into local indices
                int begin = std::ceil(row_size * range.first * (Scalar(1.0) / (Scalar(2.0) * M_PI)));
                int end   = std::ceil(row_size * range.second * (Scalar(1.0) / (Scalar(2.0) * M_PI)));

                // Floating point numbers are annoying... did you know pi * 1/pi is slightly larger than 1?
                // It's also possible that our theta ranges cross the wrap around but the indices mean they don't
                // This will cause segfaults unless we fix the wrap
                begin = begin > row_size ? 0 : begin;
                end   = end > row_size ? row_size : end;

                // If we define an empty range don't bother doing any more
                if (begin != end) {
                    // If we define a nice enclosed range range add it
                    if (begin < end) {
                        indices.emplace_back(row.begin + begin, row.begin + end);
                    }
                    // Our phi values wrap around so we need two ranges
                    else {
                        indices.emplace_back(row.begin, row.begin + end);
                        indices.emplace_back(row.begin + begin, row.end);
                    }
                }
            }
        }

        return {mesh, indices};
    }

    std::pair<const Mesh&, std::vector<std::pair<uint, uint>>> lookup(const mat4& Hoc, const Lens& lens) {

        // We multiply a lot of things by 2
        constexpr const Scalar x2 = Scalar(2.0);

        // Cut down how many points we send here by calculating how many will be on screen
        switch (lens.projection) {
            case Lens::RECTILINEAR: {

                // Extract our rotation matrix
                const mat3 Roc = {{
                    {{Hoc[0][0], Hoc[0][1], Hoc[0][2]}},  //
                    {{Hoc[1][0], Hoc[1][1], Hoc[1][2]}},  //
                    {{Hoc[2][0], Hoc[2][1], Hoc[2][2]}}   //
                }};

                // The height of our camera above the observation plane
                const Scalar& height = Hoc[2][3];

                // Print our camera vector
                const std::array<Scalar, 3> cam = {{Hoc[0][0], Hoc[1][0], Hoc[2][0]}};

                // Work out how much additional y and z we get from our field of view if we have a focal length of 1
                const Scalar y_extent = std::tan(lens.fov * Scalar(0.5));
                const Scalar z_extent = y_extent * Scalar(lens.dimensions[1]) / Scalar(lens.dimensions[0]);

                /* The labels for each of the corners of the frustum is shown below.
                    ^    T       U
                    |        C
                    z    W       V
                    <- y
                 */
                // Make vectors to the corners in cam space
                const std::array<vec3, 4> rNCc = {{
                    {{Scalar(1.0), +y_extent, +z_extent}},  // rTCc
                    {{Scalar(1.0), -y_extent, +z_extent}},  // rUCc
                    {{Scalar(1.0), -y_extent, -z_extent}},  // rVCc
                    {{Scalar(1.0), +y_extent, -z_extent}}   // rWCc
                }};

                // Rotate these into world space by multiplying by the rotation matrix
                const std::array<vec3, 4> rNCo = {{
                    {{dot(rNCc[0], Roc[0]), dot(rNCc[0], Roc[1]), dot(rNCc[0], Roc[2])}},  // rTCo
                    {{dot(rNCc[1], Roc[0]), dot(rNCc[1], Roc[1]), dot(rNCc[1], Roc[2])}},  // rUCo
                    {{dot(rNCc[2], Roc[0]), dot(rNCc[2], Roc[1]), dot(rNCc[2], Roc[2])}},  // rVCo
                    {{dot(rNCc[3], Roc[0]), dot(rNCc[3], Roc[1]), dot(rNCc[3], Roc[2])}},  // rWCo
                }};

                // Make our corner to next corner vectors
                // In cam space these are 0,1,0 style vectors so we just get a col of the other matrix
                // But since we are multiplying by the transpose we get a row of the matrix
                // When we are storing this matrix we represent each corner as N and the following clockwise corner
                // as M Then it is multiplied by the extent to make a vector of the length of the edge of the
                // frustum
                const std::array<vec3, 4> rMNo = {{
                    {{-Roc[0][1] * x2 * y_extent, -Roc[1][1] * x2 * y_extent, -Roc[2][1] * x2 * y_extent}},  // rUTo
                    {{-Roc[0][2] * x2 * z_extent, -Roc[1][2] * x2 * z_extent, -Roc[2][2] * x2 * z_extent}},  // rVUo
                    {{+Roc[0][1] * x2 * y_extent, +Roc[1][1] * x2 * y_extent, +Roc[2][1] * x2 * y_extent}},  // rWVo
                    {{+Roc[0][2] * x2 * z_extent, +Roc[1][2] * x2 * z_extent, +Roc[2][2] * x2 * z_extent}}   // rTWo
                }};

                // Make our normals to the frustum edges
                const std::array<vec3, 4> edges = {{
                    cross(rNCo[0], rNCo[1]),  // Top edge
                    cross(rNCo[1], rNCo[2]),  // Left edge
                    cross(rNCo[2], rNCo[3]),  // Base edge
                    cross(rNCo[3], rNCo[0]),  // Right edge
                }};

                // These calculations are intermediates for the solution to the cone/line equation. Since these
                // parts are the same for all phi values, we can pre-calculate them here to save effort later
                std::array<std::array<Scalar, 6>, 4> eq_parts;
                for (int i = 0; i < 4; ++i) {
                    const auto& o = rNCo[i];  // Line origin
                    const auto& d = rMNo[i];  // Line direction

                    // Later we will use these constants like so
                    // (p[0] + c2 * p[1] ± sqrt(c2 * p[2] + p[3]))/(p[4] + c2 * p[5]);

                    // c2 dependant part of numerator
                    eq_parts[i][0] = d[2] * o[2];  // -dz oz

                    // Non c2 dependant part of numerator
                    eq_parts[i][1] = -d[1] * o[1] - d[0] * o[0];  // -dy oy - dx ox

                    // c2 dependant part of discriminant
                    eq_parts[i][2] = d[0] * d[0] * o[2] * o[2]         // dx^2 oz^2
                                     - x2 * d[0] * d[2] * o[0] * o[2]  // 2 dx dz ox oz
                                     + d[1] * d[1] * o[2] * o[2]       // dy^2 oz^2
                                     - x2 * d[1] * d[2] * o[1] * o[2]  // 2 dy dz oy oz
                                     + d[2] * d[2] * o[0] * o[0]       // d_z^2 o_x^2
                                     + d[2] * d[2] * o[1] * o[1];      // d_z^2 o_y^2

                    // non c2 dependant part of discriminant
                    eq_parts[i][3] = -d[0] * d[0] * o[1] * o[1]        // dx^2 oy^2
                                     + x2 * d[0] * d[1] * o[0] * o[1]  // 2 dx dy ox oy
                                     - d[1] * d[1] * o[0] * o[0];      // dy^2 ox^2

                    // c2 dependant part of denominator
                    eq_parts[i][4] = -d[2] * d[2];  // -(dz^2)

                    // non c2 dependant part of denominator
                    eq_parts[i][5] = d[0] * d[0] + d[1] * d[1];  // dx^2 + dy^2
                }

                // Calculate our theta limits
                auto theta_limits = [&](const Scalar& phi) {
                    // Precalculate some trigonometric functions
                    const Scalar sin_phi = std::sin(phi);
                    const Scalar cos_phi = std::cos(phi);
                    const Scalar tan_phi = std::tan(phi);

                    // Cone gradient squared
                    const Scalar c2 = tan_phi * tan_phi;

                    // Store any limits we find
                    std::vector<Scalar> limits;

                    // Count how many complex solutions we get
                    int complex_sols = 0;

                    for (int i = 0; i < 4; ++i) {
                        // We make a line origin + ray to define a parametric line
                        // Note that both of these vectors are always unit length
                        const auto& o = rNCo[i];  // Line origin
                        const auto& d = rMNo[i];  // Line direction

                        // Calculate the first half of our numerator
                        const Scalar num = c2 * eq_parts[i][0] + eq_parts[i][1];

                        // Calculate our discriminant.
                        const Scalar disc = c2 * eq_parts[i][2] + eq_parts[i][3];

                        // Calculate our denominator
                        const Scalar denom = c2 * eq_parts[i][4] + eq_parts[i][5];

                        // We need to count how many complex solutions we get, if all 4 are we totally enclose phi
                        // We also don't care about the case with one solution (touching an edge)
                        if (disc <= Scalar(0.0)) {
                            ++complex_sols;
                        }
                        else if (denom != Scalar(0.0)) {

                            // We have two intersections with either the upper or lower cone
                            Scalar root = std::sqrt(disc);

                            // Get our two solutions for t
                            for (const Scalar t : {(num + root) / denom, (num - root) / denom}) {

                                // Check we are within the valid range for our segment.
                                // Since we set the length of the direction vector to the length of the side we can
                                // check it's less than one
                                if (t >= Scalar(0.0) && t <= Scalar(1.0)) {

                                    // We check z first to make sure it's on the correct side
                                    const Scalar z = o[2] + d[2] * t;

                                    // If we are both above, or both below the horizon
                                    if ((z > Scalar(0.0)) == (phi > M_PI_2)) {

                                        const Scalar x     = o[0] + d[0] * t;
                                        const Scalar y     = o[1] + d[1] * t;
                                        const Scalar theta = std::atan2(y, x);
                                        // atan2 gives a result from -pi -> pi, we need 0 -> 2 pi
                                        limits.emplace_back(theta > 0 ? theta : theta + M_PI * Scalar(2.0));
                                    }
                                }
                            }
                        }
                    }

                    // If all solutions are complex we totally enclose the phi however we still need to check the
                    // cone is on the correct side
                    if (complex_sols == 4 && ((cos_phi > Scalar(0.0)) == (cam[2] < Scalar(0.0)))) {

                        // Make a test unit vector that is on the cone, theta=0 is easiest
                        const vec3 test_vec = {{sin_phi, Scalar(0.0), -cos_phi}};

                        bool external = false;
                        for (int i = 0; !external && i < 4; ++i) {
                            // If we get a negative dot product our point is external
                            external = Scalar(0.0) > dot(test_vec, edges[i]);
                        }
                        if (!external) {
                            return std::vector<std::pair<Scalar, Scalar>>(
                                1, std::make_pair(Scalar(0.0), Scalar(2.0) * M_PI));
                        }
                    }
                    // If we have intersections
                    else if (!limits.empty()) {
                        // If we have an even number of intersections
                        if (limits.size() % 2 == 0) {
                            // Sort the limits
                            std::sort(limits.begin(), limits.end());

                            // Get a test point half way between the first two points
                            const Scalar test_theta = (limits[0] + limits[1]) * Scalar(0.5);
                            const Scalar sin_theta  = std::sin(test_theta);
                            const Scalar cos_theta  = std::cos(test_theta);

                            // Make a unit vector from the phi and theta
                            const vec3 test_vec = {{cos_theta * sin_phi, sin_theta * sin_phi, -cos_phi}};

                            bool first_is_end = false;
                            for (int i = 0; !first_is_end && i < 4; ++i) {
                                // If we get a negative dot product our first point is an end segment
                                first_is_end = Scalar(0.0) > dot(test_vec, edges[i]);
                            }

                            // If this is entering, point 0 is a start, and point 1 is an end
                            std::vector<std::pair<Scalar, Scalar>> output;
                            for (uint i = first_is_end ? 1 : 0; i + 1 < limits.size(); i += 2) {
                                output.emplace_back(limits[i], limits[i + 1]);
                            }
                            if (first_is_end) {
                                output.emplace_back(limits.back(), limits.front());
                            }
                            return output;
                        }
                        // If we have an odd number of intersections something is wrong
                        // In this case we err on the side of caution and oversample selecting points at the widest
                        // marks
                        else {
                            throw std::runtime_error("Odd number of intersections found with cone");
                        }
                    }

                    // Default to returning an empty list
                    return std::vector<std::pair<Scalar, Scalar>>();
                };

                return lookup(height, theta_limits);
            }

            // Both the radial lenses can be treated the same here
            // This only works for full frame, otherwise there are extra points that are not removed
            case Lens::EQUIDISTANT:
            case Lens::EQUISOLID: {
                // Solution for intersections on the edge is the intersection between a unit sphere, a plane, and a
                // cone The cone is the cone made by the phi angle, and the plane intersects with the unit sphere to
                // form The circle that defines the edge of the field of view of the camera.
                //
                // Unit sphere
                // x^2 + y^2 + z^2 = 1
                //
                // Cone (don't need to check side for phi since it's squared)
                // z^2 = (x^2+y^2)/c^2
                // c = tan(phi)
                //
                // Plane
                // N = the unit vector in the direction of the camera
                // r_0 = N * cos(fov/2)
                // N . (r - r_0) = 0
                //
                // To simplify things however, we remove the y component and assume the camera vector is only ever
                // on the x/z plane. We calculate the offset to make this happen and re apply it at the end

                // The gradient of our field of view cone
                const Scalar cos_half_fov = std::cos(lens.fov * Scalar(0.5));
                const vec3 cam            = {{Hoc[0][0], Hoc[1][0], Hoc[2][0]}};

                // The height of our camera above the observation plane
                const Scalar& height = Hoc[2][3];

                auto theta_limits = [&](const Scalar& phi) -> std::array<std::pair<Scalar, Scalar>, 1> {
                    // Check if we are intersecting with an upper or lower cone
                    const bool upper = phi > M_PI_2;

                    // The cameras inclination from straight down (same reference frame as phi)
                    const Scalar cam_inc  = std::acos(-cam[2]);
                    const Scalar half_fov = lens.fov * 0.5;
                    // TODO work out if you can move these out of the lambda?

                    // First we should check if this phi is totally contained in our fov
                    // Work out what our largest fully contained phi value is
                    // We can work this out by subtracting our offset angle from our fov and checking if phi is
                    // smaller
                    if ((upper && half_fov - (M_PI - cam_inc) > M_PI - phi) || (!upper && half_fov - cam_inc > phi)) {
                        return {{std::make_pair(Scalar(0.0), Scalar(2.0) * M_PI)}};
                    }
                    // Also if we can tell that the phi is totally outside we can bail out early
                    // To check this we check phi is greater than our inclination plus our fov
                    if ((upper && half_fov + (M_PI - cam_inc) < M_PI - phi) || (!upper && half_fov + cam_inc < phi)) {
                        return {{std::make_pair(Scalar(0.0), Scalar(0.0))}};
                    }

                    // The solution only works for camera vectors that lie in the x/z plane
                    // So we have to rotate our vector into that space, solve it and then rotate them back
                    // Normally this would be somewhat unsafe as cam[1] and cam[0] could be both 0
                    // However, that case is resolved by the checks above that confirm we intersect
                    const Scalar offset     = std::atan2(cam[1], cam[0]);
                    const Scalar sin_offset = std::sin(offset);
                    const Scalar cos_offset = std::cos(offset);

                    // Now we must rotate our cam vector before doing the solution
                    // Since y will be 0, and z doesn't change we only need this one
                    const Scalar r_x = cam[0] * cos_offset + cam[1] * sin_offset;

                    // The z component of our solution
                    const Scalar z = -std::cos(phi);

                    // Calculate intermediate products
                    const Scalar a = Scalar(1.0) - z * z;  // aka sin^2(phi)
                    const Scalar x = (cos_half_fov - cam[2] * z) / r_x;

                    // The y component is ± this square root
                    const Scalar y_disc = a - x * x;

                    if (y_disc < 0) {
                        return {{std::make_pair(Scalar(0.0), Scalar(0.0))}};
                    }

                    const Scalar y  = std::sqrt(y_disc);
                    const Scalar t1 = offset + std::atan2(-y, x);
                    const Scalar t2 = offset + std::atan2(y, x);

                    return {{std::make_pair(t1 > Scalar(0.0) ? t1 : t1 + Scalar(2.0) * M_PI,
                                            t2 > Scalar(0.0) ? t2 : t2 + Scalar(2.0) * M_PI)}};
                };

                // Lookup the mesh
                return lookup(height, theta_limits);
            }
            default: { throw std::runtime_error("Unknown lens type"); }
        }
    }

    ProjectedMesh project(const mat4& Hoc, const Lens& lens) {

        // Reused variables
        cl_int error;
        cl_event ev = nullptr;

        // Timer t;  // TIMER_LINE

        // Pack Rco into a float16
        cl_float16 Rco = {Hoc[0][0],
                          Hoc[1][0],
                          Hoc[2][0],
                          Scalar(0.0),
                          Hoc[0][1],
                          Hoc[1][1],
                          Hoc[2][1],
                          Scalar(0.0),
                          Hoc[0][2],
                          Hoc[1][2],
                          Hoc[2][2],
                          Scalar(0.0),
                          Scalar(0.0),
                          Scalar(0.0),
                          Scalar(0.0),
                          Scalar(0.0)};

        // Perform our lookup to get our relevant range
        auto ranges = lookup(Hoc, lens);

        // t.measure("\tLookup Range (cpu)");  // TIMER_LINE

        // Convenience variables
        const auto& cl_points = ranges.first.cl_points;
        const auto& nodes     = ranges.first.nodes;

        // First count the size of the buffer we will need to allocate
        int points = 0;
        for (const auto& range : ranges.second) {
            points += range.second - range.first;
        }

        // No point processing if we have no points, return an empty mesh
        if (points == 0) {
            return ProjectedMesh();
        }

        // Build up our list of indices for OpenCL
        // Use iota to fill in the numbers
        std::vector<int> indices(points);
        auto it = indices.begin();
        for (const auto& range : ranges.second) {
            auto n = std::next(it, range.second - range.first);
            std::iota(it, n, range.first);
            it = n;
        }

        // t.measure("\tBuild Range (cpu)");  // TIMER_LINE

        // Create buffers for indices map
        cl::mem indices_map(::clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(cl_int) * points, nullptr, &error),
                            ::clReleaseMemObject);
        if (error) {
            throw std::system_error(error, opencl_error_category(), "Error allocating indices_map buffer");
        }
        cl::mem pixel_coordinates(
            ::clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(std::array<Scalar, 2>) * points, nullptr, &error),
            ::clReleaseMemObject);
        if (error) {
            throw std::system_error(error, opencl_error_category(), "Error allocating pixel_coordinates buffer");
        }

        // Upload our indices map
        cl::event indices_event;
        ev    = nullptr;
        error = ::clEnqueueWriteBuffer(
            queue, indices_map, false, 0, indices.size() * sizeof(cl_int), indices.data(), 0, nullptr, &ev);
        if (ev) indices_event = cl::event(ev, ::clReleaseEvent);
        if (error) {
            throw std::system_error(error, opencl_error_category(), "Error uploading indices_map to device");
        }

        // indices_event.wait();               // TIMER_LINE
        // t.measure("\tUpload Range (mem)");  // TIMER_LINE

        // When everything is uploaded, we can run our projection kernel to get the pixel coordinates
        cl::event projected;
        ev = nullptr;
        /* mutex scope */ {
            std::lock_guard<std::mutex> lock(projection_mutex);

            cl::kernel projection_kernel;

            // Select a projection kernel
            switch (lens.projection) {
                case Lens::RECTILINEAR: projection_kernel = project_rectilinear; break;
                case Lens::EQUIDISTANT: projection_kernel = project_equidistant; break;
                case Lens::EQUISOLID: projection_kernel = project_equisolid; break;
            }

            // Load the arguments
            error = ::clSetKernelArg(projection_kernel, 0, cl_points.size(), &cl_points);
            if (error != CL_SUCCESS) {
                throw std::system_error(
                    error, opencl_error_category(), "Error setting kernel argument 0 for projection kernel");
            }
            error = ::clSetKernelArg(projection_kernel, 1, indices_map.size(), &indices_map);
            if (error != CL_SUCCESS) {
                throw std::system_error(
                    error, opencl_error_category(), "Error setting kernel argument 1 for projection kernel");
            }
            error = ::clSetKernelArg(projection_kernel, 2, sizeof(cl_float16), &Rco);
            if (error != CL_SUCCESS) {
                throw std::system_error(
                    error, opencl_error_category(), "Error setting kernel argument 2 for projection kernel");
            }
            error = ::clSetKernelArg(projection_kernel, 3, sizeof(lens.focal_length), &lens.focal_length);
            if (error != CL_SUCCESS) {
                throw std::system_error(
                    error, opencl_error_category(), "Error setting kernel argument 3 for projection kernel");
            }
            error = ::clSetKernelArg(projection_kernel, 4, sizeof(lens.dimensions), lens.dimensions.data());
            if (error != CL_SUCCESS) {
                throw std::system_error(
                    error, opencl_error_category(), "Error setting kernel argument 4 for projection kernel");
            }
            error = ::clSetKernelArg(projection_kernel, 5, pixel_coordinates.size(), &pixel_coordinates);
            if (error != CL_SUCCESS) {
                throw std::system_error(
                    error, opencl_error_category(), "Error setting kernel argument 5 for projection kernel");
            }

            // Project!
            size_t offset[1]      = {0};
            size_t global_size[1] = {size_t(points)};
            error                 = ::clEnqueueNDRangeKernel(
                queue, projection_kernel, 1, offset, global_size, nullptr, 1, &indices_event, &ev);
            if (ev) projected = cl::event(ev, ::clReleaseEvent);
            if (error != CL_SUCCESS) {
                throw std::system_error(error, opencl_error_category(), "Error queueing the projection kernel");
            }
        }
        // projected.wait();                     // TIMER_LINE
        // t.measure("\tProject points (gpu)");  // TIMER_LINE

        // This can happen on the CPU while the OpenCL device is busy
        // Build the reverse lookup map where the offscreen point is one past the end
        std::vector<int> r_indices(nodes.size(), points);
        for (uint i = 0; i < indices.size(); ++i) {
            r_indices[indices[i]] = i;
        }

        // Build the packed neighbourhood map with an extra offscreen point at the end
        std::vector<std::array<int, 6>> local_neighbourhood(points + 1);
        for (uint i = 0; i < indices.size(); ++i) {
            const auto& node = nodes[indices[i]];
            for (uint j = 0; j < 6; ++j) {
                const auto& n             = node.neighbours[j];
                local_neighbourhood[i][j] = r_indices[n];
            }
        }
        // Fill in the final offscreen point which connects only to itself
        local_neighbourhood[points].fill(points);

        // t.measure("\tBuild Local Neighbourhood (cpu)");  // TIMER_LINE

        // Create buffers for local neighbourhood
        cl::mem local_n_buffer(
            ::clCreateBuffer(
                context, CL_MEM_READ_ONLY, local_neighbourhood.size() * sizeof(std::array<int, 6>), nullptr, &error),
            ::clReleaseMemObject);
        if (error) {
            throw std::system_error(error, opencl_error_category(), "Error allocating local neighbourhood buffer");
        }

        cl::event local_n_event;
        ev    = nullptr;
        error = ::clEnqueueWriteBuffer(queue,
                                       local_n_buffer,
                                       false,
                                       0,
                                       local_neighbourhood.size() * sizeof(std::array<int, 6>),
                                       local_neighbourhood.data(),
                                       0,
                                       nullptr,
                                       &ev);
        if (ev) local_n_event = cl::event(ev, ::clReleaseEvent);
        if (error) {
            throw std::system_error(error, opencl_error_category(), "Error uploading local neighbourhood to device");
        }

        // local_n_event.wait();                             // TIMER_LINE
        // t.measure("\tUpload Local Neighbourhood (mem)");  // TIMER_LINE
        ::clFlush(queue);

        return ProjectedMesh{LazyBufferReader<std::array<Scalar, 2>>(queue, pixel_coordinates, projected, points),
                             std::move(local_neighbourhood),
                             std::move(indices),
                             pixel_coordinates,
                             projected,
                             local_n_buffer,
                             local_n_event};
    }

    Classifier make_classifier(
        const std::vector<std::vector<std::pair<std::vector<std::vector<Scalar>>, std::vector<Scalar>>>>& network) {

        return Classifier(this, network);
    }

private:
    inline Scalar dot(const vec3& a, const vec3& b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    inline vec3 cross(const vec3& a, const vec3& b) {
        return {{
            a[1] * b[2] - a[2] * b[1],  // x
            a[2] * b[0] - a[0] * b[2],  // y
            a[0] * b[1] - a[1] * b[0]   // z
        }};
    }

    inline vec3 normalise(const vec3& a) {
        Scalar length = Scalar(1.0) / std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] + a[2]);
        return {{a[0] * length, a[1] * length, a[2] * length}};
    }

    std::string get_scalar_defines(float) {
        return "#define Scalar float\n"
               "#define Scalar2 float2\n"
               "#define Scalar3 float3\n"
               "#define Scalar4 float4\n"
               "#define Scalar8 float8\n"
               "#define Scalar16 float16\n";
    }

    std::string get_scalar_defines(double) {
        return "#define Scalar double\n"
               "#define Scalar2 double2\n"
               "#define Scalar3 double3\n"
               "#define Scalar4 double4\n"
               "#define Scalar8 double8\n"
               "#define Scalar16 double16\n";
    }

    void setup_opencl() {

        // Get our platforms
        cl_uint platform_count = 0;
        ::clGetPlatformIDs(0, nullptr, &platform_count);
        std::vector<cl_platform_id> platforms(platform_count);
        ::clGetPlatformIDs(platforms.size(), platforms.data(), nullptr);

        if (platform_count == 0) {
            NUClear::log<NUClear::ERROR>("No OpenCL platforms found. Check OpenCL Installation");
            throw std::runtime_error("No OpenCL platforms found. Check OpenCL Installation");
        }

        // Which device/platform we are going to use
        cl_platform_id best_platform = nullptr;
        cl_device_id best_device     = nullptr;
        int best_compute_units       = 0;

        // Go through our platforms
        for (const auto& platform : platforms) {
            cl_uint device_count = 0;
            ::clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 0, nullptr, &device_count);
            std::vector<cl_device_id> devices(device_count);
            ::clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, device_count, devices.data(), nullptr);

            // Go through our devices on the platform
            for (const auto& device : devices) {

                // Length of data for strings
                size_t len;
                std::vector<char> data;

                // Print device details
                ::clGetDeviceInfo(device, CL_DEVICE_NAME, 0, nullptr, &len);
                data.resize(len);
                ::clGetDeviceInfo(device, CL_DEVICE_NAME, len, data.data(), nullptr);
                NUClear::log<NUClear::INFO>(fmt::format("\tDevice: {}", std::string(data.begin(), data.end())));


                ::clGetDeviceInfo(device, CL_DEVICE_VERSION, 0, nullptr, &len);
                data.resize(len);
                ::clGetDeviceInfo(device, CL_DEVICE_VERSION, len, data.data(), nullptr);
                NUClear::log<NUClear::INFO>(
                    fmt::format("\tHardware version: {}", std::string(data.begin(), data.end())));


                ::clGetDeviceInfo(device, CL_DRIVER_VERSION, 0, nullptr, &len);
                data.resize(len);
                ::clGetDeviceInfo(device, CL_DRIVER_VERSION, len, data.data(), nullptr);
                NUClear::log<NUClear::INFO>(
                    fmt::format("\tSoftware version: {}", std::string(data.begin(), data.end())));


                ::clGetDeviceInfo(device, CL_DEVICE_OPENCL_C_VERSION, 0, nullptr, &len);
                data.resize(len);
                ::clGetDeviceInfo(device, CL_DEVICE_OPENCL_C_VERSION, len, data.data(), nullptr);
                NUClear::log<NUClear::INFO>(
                    fmt::format("\tOpenCL C version: {}", std::string(data.begin(), data.end())));


                cl_uint max_compute_units = 0;
                ::clGetDeviceInfo(
                    device, CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(max_compute_units), &max_compute_units, nullptr);
                NUClear::log<NUClear::INFO>(fmt::format("\tParallel compute units: {}", max_compute_units));

                if (max_compute_units > best_compute_units) {
                    best_compute_units = max_compute_units;
                    best_platform      = platform;
                    best_device        = device;
                }
            }
        }

        if ((best_platform == nullptr) || (best_device == nullptr) || (best_compute_units == 0)) {
            NUClear::log<NUClear::ERROR>("No OpenCL devices found. Check OpenCL Installation");
            throw std::runtime_error("No OpenCL devices found. Check OpenCL Installation");
        }

        // Print information about our selected device
        {
            // Length of data for strings
            size_t len;
            std::vector<char> data;

            // Print device details
            ::clGetDeviceInfo(best_device, CL_DEVICE_NAME, 0, nullptr, &len);
            NUClear::log("Length of allocation", len);
            data.resize(len);
            ::clGetDeviceInfo(best_device, CL_DEVICE_NAME, len, data.data(), nullptr);
            std::cout << "\tDevice: " << std::string(data.begin(), data.end()) << std::endl;

            ::clGetDeviceInfo(best_device, CL_DEVICE_VERSION, 0, nullptr, &len);
            data.resize(len);
            ::clGetDeviceInfo(best_device, CL_DEVICE_VERSION, len, data.data(), nullptr);
            std::cout << "\tHardware version: " << std::string(data.begin(), data.end()) << std::endl;

            ::clGetDeviceInfo(best_device, CL_DRIVER_VERSION, 0, nullptr, &len);
            data.resize(len);
            ::clGetDeviceInfo(best_device, CL_DRIVER_VERSION, len, data.data(), nullptr);
            std::cout << "\tSoftware version: " << std::string(data.begin(), data.end()) << std::endl;

            ::clGetDeviceInfo(best_device, CL_DEVICE_OPENCL_C_VERSION, 0, nullptr, &len);
            data.resize(len);
            ::clGetDeviceInfo(best_device, CL_DEVICE_OPENCL_C_VERSION, len, data.data(), nullptr);
            std::cout << "\tOpenCL C version: " << std::string(data.begin(), data.end()) << std::endl;
        }

        // Make context
        cl_int error;
        context =
            cl::context(::clCreateContext(nullptr, 1, &best_device, nullptr, nullptr, &error), ::clReleaseContext);
        if (error) {
            throw std::system_error(error, opencl_error_category(), "Error creating the OpenCL context");
        }

        // Try to make an out of order queue if we can
        queue = cl::command_queue(
            ::clCreateCommandQueue(context, best_device, CL_QUEUE_OUT_OF_ORDER_EXEC_MODE_ENABLE, &error),
            ::clReleaseCommandQueue);
        if (error == CL_INVALID_VALUE) {
            queue = cl::command_queue(::clCreateCommandQueue(context, best_device, 0, &error), ::clReleaseCommandQueue);
        }
        if (error) {
            throw std::system_error(error, opencl_error_category(), "Error creating the OpenCL command queue");
        }

        // Get program sources (this does concatenated strings)
        std::string source =
            PROJECT_EQUIDISTANT_CL PROJECT_EQUISOLID_CL PROJECT_RECTILINEAR_CL READ_IMAGE_TO_NETWORK_CL;
        source = get_scalar_defines(Scalar(0.0)) + source;

        const char* cstr = source.c_str();
        size_t csize     = source.size();

        program = cl::program(::clCreateProgramWithSource(context, 1, &cstr, &csize, &error), ::clReleaseProgram);
        if (error != CL_SUCCESS) {
            throw std::system_error(error, opencl_error_category(), "Error adding sources to projection program");
        }

        // Compile the program
        error = ::clBuildProgram(
            program, 0, nullptr, "-cl-single-precision-constant -cl-fast-relaxed-math", nullptr, nullptr);
        if (error != CL_SUCCESS) {
            // Get program build log
            size_t used = 0;
            ::clGetProgramBuildInfo(program, best_device, CL_PROGRAM_BUILD_LOG, 0, nullptr, &used);
            std::vector<char> log(used);
            ::clGetProgramBuildInfo(program, best_device, CL_PROGRAM_BUILD_LOG, log.size(), log.data(), &used);

            // Throw an error with the build log
            throw std::system_error(
                error,
                opencl_error_category(),
                "Error building projection program\n" + std::string(log.begin(), log.begin() + used));
        }

        project_rectilinear = cl::kernel(::clCreateKernel(program, "project_rectilinear", &error), ::clReleaseKernel);
        if (error != CL_SUCCESS) {
            throw std::system_error(error, opencl_error_category(), "Error getting project_rectilinear kernel");
        }
        project_equidistant = cl::kernel(::clCreateKernel(program, "project_equidistant", &error), ::clReleaseKernel);
        if (error != CL_SUCCESS) {
            throw std::system_error(error, opencl_error_category(), "Error getting project_equidistant kernel");
        }
        project_equisolid = cl::kernel(::clCreateKernel(program, "project_equisolid", &error), ::clReleaseKernel);
        if (error != CL_SUCCESS) {
            throw std::system_error(error, opencl_error_category(), "Error getting project_equisolid kernel");
        }
        read_image_to_network =
            cl::kernel(::clCreateKernel(program, "read_image_to_network", &error), ::clReleaseKernel);
        if (error != CL_SUCCESS) {
            throw std::system_error(error, opencl_error_category(), "Error getting read_image_to_network kernel");
        }
    }

    // Our OpenCL context
    cl::context context;

    // OpenCL queue for executing kernels
    cl::command_queue queue;

    // OpenCL kernel functions
    cl::program program;
    cl::kernel project_equidistant;
    cl::kernel project_equisolid;
    cl::kernel project_rectilinear;
    cl::kernel read_image_to_network;

    // Mutex to protect image read
    std::mutex read_image_to_network_mutex;

    // Mutex to protect projection functions
    std::mutex projection_mutex;

    /// A map from heights to visual mesh tables
    std::map<Scalar, Mesh> luts;

    /// The smallest angular width the LUT should be generated for
    Scalar min_angular_res;
    /// The minimum height the the luts are generated for
    Scalar min_height;
    // The maximum height the luts are generated for
    Scalar max_height;
    // The number gradations in height
    uint height_resolution;
};  // namespace mesh

}  // namespace mesh

#endif  // VISUALMESH_HPP
