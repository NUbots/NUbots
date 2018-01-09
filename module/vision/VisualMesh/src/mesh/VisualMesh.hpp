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

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <map>
#include <numeric>
#include <sstream>
#include <vector>

#define CL_HPP_MINIMUM_OPENCL_VERSION 120
#define CL_HPP_TARGET_OPENCL_VERSION 120
#define CL_HPP_ENABLE_EXCEPTIONS
#include "cl/cl2.hpp"

// Include our generated OpenCL headers
#include "mesh/cl/project_equidistant.cl.h"
#include "mesh/cl/project_equisolid.cl.h"
#include "mesh/cl/project_rectilinear.cl.h"
#include "mesh/cl/read_image_to_network.cl.h"

#include "utility/support/Timer.hpp"

namespace mesh {

template <typename T>
struct LazyBufferReader {

    LazyBufferReader() = default;

    LazyBufferReader(const cl::CommandQueue& queue, const cl::Buffer& buffer, const cl::Event& ready, uint n_elements)
        : queue(queue), buffer(buffer), ready(ready), n_elements(n_elements) {}

    operator std::vector<T>() {
        std::vector<T> output(n_elements);
        std::vector<cl::Event> ready_list = {ready};
        queue.enqueueReadBuffer(buffer, true, 0, n_elements * sizeof(T), output.data(), &ready_list);
        return output;
    }

    operator T() {
        T output;
        std::vector<cl::Event> ready_list = {ready};
        queue.enqueueReadBuffer(buffer, true, 0, sizeof(T), &output, &ready_list);
        return output;
    }

    cl::CommandQueue queue;
    cl::Buffer buffer;
    cl::Event ready;
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
        LazyBufferReader<std::array<int, 2>> pixel_coordinates;
        std::vector<std::array<int, 6>> neighbourhood;

        // OpenCL buffers for the data
        struct {
            cl::Buffer pixel_coordinates;
            cl::Event pixel_coordinates_event;
            cl::Buffer neighbourhood;
            cl::Event neighbourhood_event;
        } cl;
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
        Mesh(std::vector<Node>&& nodes, std::vector<Row>&& rows, cl::Buffer&& cl_points)
            : nodes(nodes), rows(rows), cl_points(cl_points) {}

        /// The lookup table for this mesh
        std::vector<Node> nodes;
        /// A set of individual rows for phi values. `begin` and `end` refer to the table with end being 1 past the end
        std::vector<Row> rows;

        /// The on device buffer of the visual mesh unit vectors
        cl::Buffer cl_points;
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

        Classifier(VisualMesh* mesh, const network_structure_t& structure) : mesh(mesh) {

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
                    if (vector_out) {
                        // Apply elu
                        std::string e = "in" + std::to_string(layer_no + 1);

                        // TODO Apply selu rather than elu
                        code << "    " << e << " = select(native_exp(" << e << ") - 1, in" << (layer_no + 1) << ", "
                             << e << " > 0);" << std::endl;  // select(a, b, c) == c ? b : a
                    }
                    else {
                        for (uint i = 0; i < biases.size(); ++i) {
                            std::string e = "in" + std::to_string(layer_no + 1) + "[" + std::to_string(i) + "]";
                            code << "    " << e << " = " << e << " > 0 ? " << e << " : native_exp(" << e << ") - 1;"
                                 << std::endl;
                        }
                    }
                    code << std::endl;

                    // If this is our last layer, apply softmax
                    if (conv_no + 1 == structure.size() && layer_no + 1 == conv.size()) {
                        code << "    // Apply softmax to our final output" << std::endl;

                        if (vector_out) {
                            std::string e = "in" + std::to_string(layer_no + 1);
                            code << "    " << e << " = native_exp(" << e << ");" << std::endl;
                            code << "    " << e << " = " << e << " / dot(" << e << ", (float" << vector_out << ")(1));"
                                 << std::endl;
                        }
                        else {

                            // Apply exp to each of the elements
                            for (uint i = 0; i < biases.size(); ++i) {
                                std::string e = "in" + std::to_string(layer_no + 1) + "[" + std::to_string(i) + "]";
                                code << "    " << e << " = native_exp(" << e << ");" << std::endl;
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

            // Compile the OpenCL program
            cl::Program::Sources sources({code.str()});

            // Build the program
            program = cl::Program(mesh->context, sources);

            try {
                if (program.build("-cl-single-precision-constant -cl-strict-aliasing -cl-fast-relaxed-math")
                    != CL_SUCCESS) {
                    throw std::runtime_error("Error building VisualMesh Classifier\n"
                                             + program.getBuildInfo<CL_PROGRAM_BUILD_LOG>().front().second);
                }
            }
            catch (const cl::BuildError) {
                throw std::runtime_error("Error building VisualMesh Classifier\n"
                                         + program.getBuildInfo<CL_PROGRAM_BUILD_LOG>().front().second);
            }

            for (uint i = 0; i < structure.size(); ++i) {
                std::string kernel = "conv" + std::to_string(i);
                uint output_size   = structure[i].back().second.size();
                conv_layers.emplace_back(
                    cl::KernelFunctor<const cl::Buffer&, const cl::Buffer&, cl::Buffer&>(program, kernel), output_size);
            }
        }

        std::vector<std::array<float, 2>> operator()(const void* image,
                                                     const FOURCC& format,
                                                     const mat4& Hoc,
                                                     const Lens& lens) {

            // Upload the image using the memory queue
            cl::array<size_t, 3> origin = {{0, 0, 0}};
            cl::array<size_t, 3> region = {{size_t(lens.dimensions[0]), size_t(lens.dimensions[1]), 1}};
            cl::ImageFormat fmt;
            switch (format) {
                // Bayer
                case GRBG:
                case RGGB:
                case GBRG:
                case BGGR: fmt = cl::ImageFormat(CL_R, CL_UNORM_INT8); break;
                // Oh no...
                default: throw std::runtime_error("Unsupported image format");
            }
            cl::Image2D img(mesh->context,
                            CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR,
                            fmt,
                            size_t(lens.dimensions[0]),
                            size_t(lens.dimensions[1]),
                            0,
                            const_cast<void*>(image));
            std::size_t row_pitch = 0;
            cl::Event img_event;
            mesh->queue.enqueueMapImage(
                img, CL_FALSE, CL_MAP_READ, origin, region, &row_pitch, nullptr, nullptr, &img_event);

            // Project our visual mesh
            auto projection = mesh->project(Hoc, lens);

            // This includes the offscreen point at the end
            int points = projection.neighbourhood.size();

            // Our buffers for each layer
            std::vector<std::pair<cl::Buffer, std::vector<cl::Event>>> layer_buffers;

            // First layer, output from the image
            cl::Buffer img_load_buffer(mesh->context, CL_MEM_READ_WRITE, sizeof(cl_float4) * points);

            // Zero out the final value in the buffer
            cl::Event byte_fill_event;
            mesh->queue.enqueueFillBuffer(img_load_buffer,
                                          cl_float4{{0.0f, 0.0f, 0.0f, 0.0f}},
                                          (points - 1) * sizeof(cl_float4),
                                          sizeof(cl_float4),
                                          nullptr,
                                          &byte_fill_event);

            // Read the pixels into the buffer
            cl::Event img_load_event = mesh->read_image_to_network(
                cl::EnqueueArgs(mesh->queue,
                                std::vector<cl::Event>({projection.cl.pixel_coordinates_event, img_event}),
                                cl::NDRange(points - 1)),  // -1 as we don't project the offscreen point
                img,
                format,
                projection.cl.pixel_coordinates,
                img_load_buffer);

            // These make up our first buffers
            layer_buffers.emplace_back(
                img_load_buffer,
                std::vector<cl::Event>({img_load_event, byte_fill_event, projection.cl.neighbourhood_event}));

            // Run each of our conv layers
            for (auto& conv : conv_layers) {
                cl::Buffer out_buffer(mesh->context, CL_MEM_READ_WRITE, size_t(conv.second * points * sizeof(float)));

                cl::Event event =
                    conv.first(cl::EnqueueArgs(mesh->queue, layer_buffers.back().second, cl::NDRange(points)),
                               projection.cl.neighbourhood,
                               layer_buffers.back().first,
                               out_buffer);

                layer_buffers.emplace_back(out_buffer, std::vector<cl::Event>({event}));
            }

            // Read out the final layers output
            std::vector<std::array<float, 2>> output(points);
            std::vector<cl::Event> ready_list = {layer_buffers.back().second};
            mesh->queue.enqueueReadBuffer(
                layer_buffers.back().first, true, 0, points * sizeof(cl_float2), output.data(), &ready_list);

            return output;
        }

    private:
        VisualMesh* mesh;
        cl::Program program;
        std::vector<std::pair<cl::KernelFunctor<const cl::Buffer&,  // The int* neighbourhood map
                                                const cl::Buffer&,  // The input buffer for the layer
                                                cl::Buffer&>,       // The output buffer for the layer
                              int>>
            conv_layers;
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

            // Loop from directly down up to the horizon (if phi is nan it will stop)
            // So we don't have a single point at the base, we move half a jump forward
            for (Scalar phi = shape.phi(Scalar(0.0), h) * Scalar(0.5); phi < M_PI_2;) {

                // Calculate our theta
                Scalar theta = std::max(shape.theta(phi, h), min_angular_res);

                if (!std::isnan(theta)) {
                    // Push back the phi, and the number of whole shapes we can fit
                    phis.emplace_back(phi, uint(std::ceil(Scalar(2.0) * M_PI / theta)));
                }

                // Move to our next phi
                phi = std::max(phi + min_angular_res, shape.phi(phi, h));
            }

            // Loop from directly up down to the horizon (if phi is nan it will stop)
            for (Scalar phi = (M_PI + shape.phi(M_PI, h)) * Scalar(0.5); phi > M_PI_2;) {

                // Calculate our theta
                Scalar theta = std::max(shape.theta(phi, h), min_angular_res);

                if (!std::isnan(theta)) {
                    // Push back the phi, and the number of whole shapes we can fit
                    phis.emplace_back(phi, uint(std::ceil(Scalar(2.0) * M_PI / theta)));
                }

                // Move to our next phi
                phi = std::min(phi - min_angular_res, shape.phi(phi, h));
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

                    // Set these two neighbours
                    n.neighbours[2] = l - i;  // L
                    n.neighbours[3] = r - i;  // R

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

                const auto& back    = rows.back();
                const int back_size = back.end - back.begin;

                // Link the front to itself
                for (int i = front.begin; i < front.end; ++i) {
                    // Alias our node
                    auto& node = lut[i];

                    // Work out which two points are on the opposite side to us
                    const uint index = i - front.begin + (front_size / 2);

                    // Find where we are in our row as a value between 0 and 1
                    const Scalar pos = Scalar(i - front.begin) / Scalar(front_size);

                    // Link to ourself
                    node.neighbours[0] = front.begin + (index % front_size) - i;
                    node.neighbours[1] = front.begin + ((index + 1) % front_size) - i;

                    // Link to our next row normally
                    const auto& r2 = rows[1];
                    link(lut, i, pos, r2.begin, r2.end - r2.begin, 4);
                }

                // Link the back to itself
                for (int i = back.begin; i < back.end; ++i) {
                    // Alias our node
                    auto& node = lut[i];

                    // Work out which two points are on the opposite side to us
                    const uint index = i - back.begin + (back_size / 2);

                    // Find where we are in our row as a value between 0 and 1
                    const Scalar pos = Scalar(i - back.begin) / Scalar(back_size);

                    // Link to ourself on the other side
                    node.neighbours[4] = back.begin + (index % back_size) - i;
                    node.neighbours[5] = back.begin + ((index + 1) % back_size) - i;

                    // Link to our previous row normally
                    const auto& r2 = rows[rows.size() - 2];
                    link(lut, i, pos, r2.begin, r2.end - r2.begin, 0);
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
            cl::Buffer cl_points_buffer(context, cl_points.begin(), cl_points.end(), true);

            // Insert our constructed mesh into the lookup
            luts.insert(std::make_pair(h, Mesh(std::move(lut), std::move(rows), std::move(cl_points_buffer))));
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

        // Timer t;  // TIMER_LINE

        // Build Rco by transposing the rotation of Hoc and upload it to the device
        const mat4 Rco = {{
            {{Hoc[0][0], Hoc[1][0], Hoc[2][0], Scalar(0.0)}},       //
            {{Hoc[0][1], Hoc[1][1], Hoc[2][1], Scalar(0.0)}},       //
            {{Hoc[0][2], Hoc[1][2], Hoc[2][2], Scalar(0.0)}},       //
            {{Scalar(0.0), Scalar(0.0), Scalar(0.0), Scalar(0.0)}}  //
        }};

        cl::Event Rco_event;
        cl::Buffer Rco_buffer(context, CL_MEM_READ_ONLY, sizeof(Rco));
        queue.enqueueWriteBuffer(Rco_buffer, false, 0, sizeof(Rco), Rco.data(), nullptr, &Rco_event);

        // Rco_event.wait();                 // TIMER_LINE
        // t.measure("\tUpload Rco (mem)");  // TIMER_LINE

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
        cl::Buffer indices_map(context, CL_MEM_READ_ONLY, sizeof(cl_int) * points);
        cl::Buffer pixel_coordinates(context, 0, sizeof(cl_int2) * points);

        // Upload our indices map
        cl::Event indices_event;
        queue.enqueueWriteBuffer(
            indices_map, false, 0, indices.size() * sizeof(cl_int), indices.data(), nullptr, &indices_event);

        // indices_event.wait();               // TIMER_LINE
        // t.measure("\tUpload Range (mem)");  // TIMER_LINE

        // When everything is uploaded, we can run our projection kernel to get the pixel coordinates
        cl::Event projected;
        switch (lens.projection) {
            case Lens::RECTILINEAR: {
                projected = project_rectilinear(
                    cl::EnqueueArgs(queue, std::vector<cl::Event>({Rco_event, indices_event}), cl::NDRange(points)),
                    cl_points,
                    indices_map,
                    Rco_buffer,
                    lens.focal_length,
                    lens.dimensions,
                    pixel_coordinates);

            } break;
            case Lens::EQUIDISTANT: {
                projected = project_equidistant(
                    cl::EnqueueArgs(queue, std::vector<cl::Event>({Rco_event, indices_event}), cl::NDRange(points)),
                    cl_points,
                    indices_map,
                    Rco_buffer,
                    lens.focal_length,
                    lens.dimensions,
                    pixel_coordinates);
            } break;
            case Lens::EQUISOLID: {
                projected = project_equisolid(
                    cl::EnqueueArgs(queue, std::vector<cl::Event>({Rco_event, indices_event}), cl::NDRange(points)),
                    cl_points,
                    indices_map,
                    Rco_buffer,
                    lens.focal_length,
                    lens.dimensions,
                    pixel_coordinates);
            } break;
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
            for (uint j = 0; j < 6; ++j) {
                const auto& n             = nodes[indices[i]].neighbours[j];
                local_neighbourhood[i][j] = r_indices[n];
            }
        }
        // Fill in the final offscreen point which connects only to itself
        local_neighbourhood[points] = {{points, points, points, points, points, points}};

        // t.measure("\tBuild Local Neighbourhood (cpu)");  // TIMER_LINE

        cl::Event local_n_event;
        cl::Buffer local_n_buffer(context, CL_MEM_READ_ONLY, local_neighbourhood.size() * sizeof(int) * 6);
        queue.enqueueWriteBuffer(local_n_buffer,
                                 false,
                                 0,
                                 local_neighbourhood.size() * sizeof(int) * 6,
                                 local_neighbourhood.data(),
                                 nullptr,
                                 &local_n_event);

        projected.wait();
        // local_n_event.wait();                             // TIMER_LINE
        // t.measure("\tUpload Local Neighbourhood (mem)");  // TIMER_LINE

        return ProjectedMesh{LazyBufferReader<std::array<int, 2>>(queue, pixel_coordinates, projected, points),
                             local_neighbourhood,
                             {pixel_coordinates, projected, local_n_buffer, local_n_event}};
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
        return "#define Scalar float\n#define Scalar2 float2\n#define Scalar3 float3\n#define Scalar4 float4\n";
    }

    std::string get_scalar_defines(double) {
        return "#define Scalar double\n#define Scalar2 double2\n#define Scalar3 double3\n#define Scalar4 double4\n";
    }

    void setup_opencl() {
        // Get all available platforms (drivers)
        std::vector<cl::Platform> all_platforms;
        cl::Platform::get(&all_platforms);
        if (all_platforms.empty()) {
            throw std::runtime_error("No OpenCL platforms found. Check OpenCL Installation");
        }

        // Chose our default platform
        cl::Platform default_platform = all_platforms.front();
        std::cerr << "Using OpenCL platform: " << default_platform.getInfo<CL_PLATFORM_NAME>() << " "
                  << default_platform.getInfo<CL_PLATFORM_VERSION>() << std::endl;

        // Get the default device of the default platform
        std::vector<cl::Device> all_devices;
        default_platform.getDevices(CL_DEVICE_TYPE_GPU, &all_devices);
        if (all_devices.empty()) {
            throw std::runtime_error("No devices found. Check OpenCL installation!");
        }

        // Choose our default device
        cl::Device default_device = all_devices.front();
        std::cerr << "Using OpenCL device: " << default_device.getInfo<CL_DEVICE_NAME>() << std::endl;

        // Make a context for this device
        context = cl::Context({default_device});

        // Create two queues, one for memory transfers and one for execution
        queue = cl::CommandQueue(context, default_device);
        queue = cl::CommandQueue(context, default_device);

        // Get our program source code
        cl::Program::Sources sources;

        // First we define our templated types
        sources.emplace_back(get_scalar_defines(Scalar(0.0)));

        // Add our sources
        sources.emplace_back(PROJECT_EQUIDISTANT_CL);
        sources.emplace_back(PROJECT_EQUISOLID_CL);
        sources.emplace_back(PROJECT_RECTILINEAR_CL);
        sources.emplace_back(READ_IMAGE_TO_NETWORK_CL);

        // Build the program
        cl::Program program(context, sources);
        try {
            if (program.build() != CL_SUCCESS) {
                throw std::runtime_error("Error building Mesh Programs\n"
                                         + program.getBuildInfo<CL_PROGRAM_BUILD_LOG>().front().second);
            }
        }
        catch (const cl::BuildError) {
            throw std::runtime_error("Error building Mesh Programs\n"
                                     + program.getBuildInfo<CL_PROGRAM_BUILD_LOG>().front().second);
        }

        using ProjectionKernelFuctor = cl::KernelFunctor<const cl::Buffer&,          // The Scalar4* unit vectors
                                                         const cl::Buffer&,          // The int* index map
                                                         const cl::Buffer&,          // The Rco matrix
                                                         const Scalar&,              // The focal length in pixels
                                                         const std::array<int, 2>&,  // The image dimensions
                                                         cl::Buffer&>;               // The output int2 buffer

        // Get our projection functions
        project_equidistant = ProjectionKernelFuctor(program, "project_equidistant");
        project_equisolid   = ProjectionKernelFuctor(program, "project_equisolid");
        project_rectilinear = ProjectionKernelFuctor(program, "project_rectilinear");

        // Build functors for reading images into the neural network layer
        read_image_to_network = cl::KernelFunctor<const cl::Image2D&,  // The image buffer
                                                  const FOURCC&,       // The binary format the image is in
                                                  const cl::Buffer&,   // The pixel coordinates to lookup
                                                  cl::Buffer&>  // The neural network layer information to output to
            (program, "read_image_to_network");
    }

    // Our OpenCL context
    cl::Context context;

    // OpenCL queue for executing kernels
    cl::CommandQueue queue;

    using ProjectionFunction = std::function<cl::Event(const cl::EnqueueArgs&,     // The number of workers to spawn etc
                                                       const cl::Buffer&,          // The Scalar4* unit vectors
                                                       const cl::Buffer&,          // The int* index map
                                                       const cl::Buffer&,          // The Rco matrix
                                                       const Scalar&,              // The focal length in pixels
                                                       const std::array<int, 2>&,  // The image dimensions
                                                       cl::Buffer&)>;              // The output int2 buffer


    // OpenCL kernel functions
    ProjectionFunction project_equidistant;
    ProjectionFunction project_equisolid;
    ProjectionFunction project_rectilinear;

    std::function<cl::Event(const cl::EnqueueArgs&,  // The number of workers to spawn etc
                            const cl::Image2D&,      // The image buffer
                            const FOURCC&,           // The binary format the image is in
                            const cl::Buffer&,       // The pixel coordinates to lookup
                            cl::Buffer&)>            // The neural network layer information to output to
        read_image_to_network;

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
};

}  // namespace mesh

#endif  // VISUALMESH_HPP
