/*
 * Copyright (C) 2017-2018 Trent Houliston <trent@houliston.me>
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

#ifndef VISUALMESH_ENGINE_OPENCL_ENGINE_HPP
#define VISUALMESH_ENGINE_OPENCL_ENGINE_HPP

#define CL_USE_DEPRECATED_OPENCL_1_2_APIS
#if defined(__APPLE__) || defined(__MACOSX)
#  include <OpenCL/opencl.h>
#else
#  include <CL/opencl.h>
#endif  // !__APPLE__

#include <numeric>
#include <tuple>
#include "engine/opencl/classifier.hpp"
#include "engine/opencl/kernels/project_equidistant.cl.hpp"
#include "engine/opencl/kernels/project_equisolid.cl.hpp"
#include "engine/opencl/kernels/project_rectilinear.cl.hpp"
#include "engine/opencl/util.hpp"
#include "engine/opencl/wrapper.hpp"
#include "mesh/mesh.hpp"
#include "mesh/network_structure.hpp"
#include "mesh/projected_mesh.hpp"
#include "opencl_error_category.hpp"
#include "util/math.hpp"

namespace visualmesh {
namespace engine {
  namespace opencl {

    template <typename Scalar>
    class Classifier;

    template <typename Scalar>
    class Engine {
    public:
      Engine() {

        // Get our platforms
        cl_uint platform_count = 0;
        ::clGetPlatformIDs(0, nullptr, &platform_count);
        std::vector<cl_platform_id> platforms(platform_count);
        ::clGetPlatformIDs(platforms.size(), platforms.data(), nullptr);

        // Which device/platform we are going to use
        cl_platform_id best_platform = nullptr;
        cl_device_id best_device     = nullptr;
        int best_compute_units       = 0;

        // Go through our platforms
        // for (const auto& platform : platforms) {
        const auto& platform = platforms.front();

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
          std::cout << "\tDevice: " << std::string(data.begin(), data.end()) << std::endl;

          ::clGetDeviceInfo(device, CL_DEVICE_VERSION, 0, nullptr, &len);
          data.resize(len);
          ::clGetDeviceInfo(device, CL_DEVICE_VERSION, len, data.data(), nullptr);
          std::cout << "\tHardware version: " << std::string(data.begin(), data.end()) << std::endl;

          ::clGetDeviceInfo(device, CL_DRIVER_VERSION, 0, nullptr, &len);
          data.resize(len);
          ::clGetDeviceInfo(device, CL_DRIVER_VERSION, len, data.data(), nullptr);
          std::cout << "\tSoftware version: " << std::string(data.begin(), data.end()) << std::endl;

          ::clGetDeviceInfo(device, CL_DEVICE_OPENCL_C_VERSION, 0, nullptr, &len);
          data.resize(len);
          ::clGetDeviceInfo(device, CL_DEVICE_OPENCL_C_VERSION, len, data.data(), nullptr);
          std::cout << "\tOpenCL C version: " << std::string(data.begin(), data.end()) << std::endl;

          cl_uint max_compute_units = 0;
          ::clGetDeviceInfo(
            device, CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(max_compute_units), &max_compute_units, nullptr);
          std::cout << "\tParallel compute units: " << max_compute_units << std::endl;

          if (max_compute_units > best_compute_units) {
            best_compute_units = max_compute_units;
            best_platform      = platform;
            best_device        = device;
          }

          std::cout << std::endl;
        }

        // Print information about our selected device
        {
          // Length of data for strings
          size_t len;
          std::vector<char> data;

          // Print device details
          ::clGetDeviceInfo(best_device, CL_DEVICE_NAME, 0, nullptr, &len);
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
        if (error) { throw std::system_error(error, opencl_error_category(), "Error creating the OpenCL context"); }

        // Try to make an out of order queue if we can
        queue = make_queue(context, best_device);

        // Get program sources (this does concatenated strings)
        std::string source = std::string(get_scalar_defines(Scalar(0.0)))
                             + PROJECT_EQUIDISTANT_CL PROJECT_EQUISOLID_CL PROJECT_RECTILINEAR_CL;

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
          throw std::system_error(error,
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
      }

      ProjectedMesh<Scalar> project(const Mesh<Scalar>& mesh,
                                    const std::vector<std::pair<unsigned int, unsigned int>>& ranges,
                                    const mat4<Scalar>& Hoc,
                                    const Lens<Scalar>& lens) const {

        std::vector<std::array<int, 6>> neighbourhood;
        std::vector<int> indices;
        cl::mem cl_pixels;
        cl::event projected;

        std::tie(neighbourhood, indices, cl_pixels, projected) = do_project(mesh, ranges, Hoc, lens);

        // Read the pixels off the buffer
        std::vector<std::array<Scalar, 2>> pixels(indices.size());
        std::vector<cl_event> events({projected});
        cl_int error = ::clEnqueueReadBuffer(queue,
                                             cl_pixels,
                                             true,
                                             0,
                                             indices.size() * sizeof(std::array<Scalar, 2>),
                                             pixels.data(),
                                             events.size(),
                                             events.data(),
                                             nullptr);
        throw_cl_error(error, "Failed reading projected pixels from the device");

        return ProjectedMesh<Scalar>{std::move(pixels), std::move(neighbourhood), std::move(indices)};
      }

      auto make_classifier(const network_structure_t<Scalar>& structure) {
        return Classifier<Scalar>(this, structure);
      }

      void clear_cache() {
        device_points_cache.clear();
      }

    private:
      std::tuple<std::vector<std::array<int, 6>>, std::vector<int>, cl::mem, cl::event> do_project(
        const Mesh<Scalar>& mesh,
        const std::vector<std::pair<unsigned int, unsigned int>>& ranges,
        const mat4<Scalar>& Hoc,
        const Lens<Scalar>& lens) const {

        // Reused variables
        cl_int error;
        cl_event ev = nullptr;

        // Pack Rco into a Scalar16
        // clang-format off
          std::array<Scalar, 16> Rco{{
              Hoc[0][0],     Hoc[1][0],   Hoc[2][0], Scalar(0.0),
              Hoc[0][1],     Hoc[1][1],   Hoc[2][1], Scalar(0.0),
              Hoc[0][2],     Hoc[1][2],   Hoc[2][2], Scalar(0.0),
              Scalar(0.0), Scalar(0.0), Scalar(0.0), Scalar(1.0)
          }};
        // clang-format on

        // Convenience variables
        const auto& nodes = mesh.nodes;

        // Upload our visual mesh unit vectors if we have to
        cl::mem cl_points;

        auto device_mesh = device_points_cache.find(&mesh);
        if (device_mesh == device_points_cache.end()) {
          cl_points = cl::mem(
            ::clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(vec4<Scalar>) * mesh.nodes.size(), nullptr, &error),
            ::clReleaseMemObject);

          // Flatten our rays
          std::vector<std::array<Scalar, 4>> rays;
          rays.reserve(mesh.nodes.size());
          for (const auto& n : mesh.nodes) {
            rays.push_back(n.ray);
          }

          // Write the points buffer to the device and cache it
          error = ::clEnqueueWriteBuffer(queue,
                                         cl_points,
                                         true,
                                         0,
                                         mesh.nodes.size() * sizeof(std::array<Scalar, 4>),
                                         rays.data(),
                                         0,
                                         nullptr,
                                         nullptr);
          throw_cl_error(error, "Error writing points to the device buffer");

          // Cache for future runs
          std::lock_guard<std::mutex> lock(cache_mutex);
          device_points_cache[&mesh] = cl_points;
        }
        else {
          cl_points = device_mesh->second;
        }

        // First count the size of the buffer we will need to allocate
        int points = 0;
        for (const auto& range : ranges) {
          points += range.second - range.first;
        }

        // No point processing if we have no points, return an empty mesh
        if (points == 0) {
          return std::make_tuple(std::vector<std::array<int, 6>>(), std::vector<int>(), cl::mem(), cl::event());
        }

        // Build up our list of indices for OpenCL
        // Use iota to fill in the numbers
        std::vector<int> indices(points);
        auto it = indices.begin();
        for (const auto& range : ranges) {
          auto n = std::next(it, range.second - range.first);
          std::iota(it, n, range.first);
          it = n;
        }

        // Create buffers for indices map
        cl::mem indices_map(::clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(cl_int) * points, nullptr, &error),
                            ::clReleaseMemObject);
        throw_cl_error(error, "Error allocating indices_map buffer");
        cl::mem pixel_coordinates(
          ::clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(std::array<Scalar, 2>) * points, nullptr, &error),
          ::clReleaseMemObject);
        throw_cl_error(error, "Error allocating pixel_coordinates buffer");

        // Upload our indices map
        cl::event indices_event;
        ev    = nullptr;
        error = ::clEnqueueWriteBuffer(
          queue, indices_map, false, 0, indices.size() * sizeof(cl_int), indices.data(), 0, nullptr, &ev);
        if (ev) indices_event = cl::event(ev, ::clReleaseEvent);
        throw_cl_error(error, "Error uploading indices_map to device");

        // When everything is uploaded, we can run our projection kernel to get the pixel coordinates
        cl::event projected;
        /* mutex scope */ {
          std::lock_guard<std::mutex> lock(projection_mutex);

          cl::kernel projection_kernel;

          // Select a projection kernel
          switch (lens.projection) {
            case RECTILINEAR: projection_kernel = project_rectilinear; break;
            case EQUIDISTANT: projection_kernel = project_equidistant; break;
            case EQUISOLID: projection_kernel = project_equisolid; break;
          }

          // Load the arguments
          cl_mem arg;
          arg   = cl_points;
          error = ::clSetKernelArg(projection_kernel, 0, sizeof(arg), &arg);
          throw_cl_error(error, "Error setting kernel argument 0 for projection kernel");
          arg   = indices_map;
          error = ::clSetKernelArg(projection_kernel, 1, sizeof(arg), &arg);
          throw_cl_error(error, "Error setting kernel argument 1 for projection kernel");
          error = ::clSetKernelArg(projection_kernel, 2, sizeof(Rco), Rco.data());
          throw_cl_error(error, "Error setting kernel argument 2 for projection kernel");
          error = ::clSetKernelArg(projection_kernel, 3, sizeof(lens.focal_length), &lens.focal_length);
          throw_cl_error(error, "Error setting kernel argument 3 for projection kernel");
          error = ::clSetKernelArg(projection_kernel, 4, sizeof(lens.dimensions), lens.dimensions.data());
          throw_cl_error(error, "Error setting kernel argument 4 for projection kernel");
          error = ::clSetKernelArg(projection_kernel, 5, sizeof(lens.centre), lens.centre.data());
          throw_cl_error(error, "Error setting kernel argument 5 for projection kernel");
          arg   = pixel_coordinates;
          error = ::clSetKernelArg(projection_kernel, 6, sizeof(arg), &arg);
          throw_cl_error(error, "Error setting kernel argument 6 for projection kernel");

          // Project!
          size_t offset[1]      = {0};
          size_t global_size[1] = {size_t(points)};
          ev                    = nullptr;
          cl_event iev          = indices_event;
          error = ::clEnqueueNDRangeKernel(queue, projection_kernel, 1, offset, global_size, nullptr, 1, &iev, &ev);
          if (ev) projected = cl::event(ev, ::clReleaseEvent);
          throw_cl_error(error, "Error queueing the projection kernel");
        }

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

        // This ensures that all elements in the queue have been issued to the device NOT that they are all finished
        // If we don't do this here, some of our buffers can go out of scope before the queue picks them up causing
        // errors
        ::clFlush(queue);

        // Return what we calculated
        return std::make_tuple(std::move(local_neighbourhood),  // CPU buffer
                               std::move(indices),              // CPU buffer
                               pixel_coordinates,               // GPU buffer
                               projected);                      // GPU event
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
      // Mutex to protect projection functions
      mutable std::mutex projection_mutex;

      // Cache of opencl buffers from mesh objects
      mutable std::map<const Mesh<Scalar>*, cl::mem> device_points_cache;
      // A mutex to protect the cache
      mutable std::mutex cache_mutex;

      friend class Classifier<Scalar>;
    };

  }  // namespace opencl
}  // namespace engine
}  // namespace visualmesh

#endif  // VISUALMESH_ENGINE_OPENCL_ENGINE_HPP
