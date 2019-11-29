#include "mosaic_to_surface.h"

#include <fmt/format.h>

#include <array>

#include "opencl_error_category.h"

namespace module::output::compressor::vaapi::cl {

void mosaic_to_surface(CompressionContext::OpenCLContext context,
                       const cl::command_queue& command_queue,
                       const cl::kernel& mosaic,
                       const cl::mem image,
                       const std::vector<uint8_t>& data,
                       const uint32_t& width,
                       const uint32_t& height,
                       const cl_mem& surface) {

    cl_int error;
    cl_event ev;

    // Initialise the description of the image so we can copy it to the device
    std::array<size_t, 3> origin = {{0, 0, 0}};
    std::array<size_t, 3> region = {{size_t(width), size_t(height), 1}};

    // Upload our image to device memory
    error = clEnqueueWriteImage(
        command_queue, image, false, origin.data(), region.data(), 0, 0, data.data(), 0, nullptr, &ev);
    cl::event copied_ev(ev, ::clReleaseEvent);
    if (error != CL_SUCCESS) {
        throw std::system_error(error, cl::opencl_error_category(), "Error mapping the image into device memory");
    }

    // Acquire the surface from VAAPI
    error = context.acquire_surface(command_queue, 1, &surface, 0, nullptr, &ev);
    cl::event acquired_ev(ev, ::clReleaseEvent);
    if (error != CL_SUCCESS) {
        throw std::system_error(error, cl::opencl_error_category(), "Error acquiring the VAAPI surface");
    }

    // Run the opencl kernel
    cl_mem arg = image;
    error      = ::clSetKernelArg(mosaic, 0, sizeof(arg), &arg);
    if (error != CL_SUCCESS) {
        throw std::system_error(error, cl::opencl_error_category(), "Error setting the first kernel argument");
    }
    arg   = surface;
    error = ::clSetKernelArg(mosaic, 1, sizeof(arg), &arg);
    if (error != CL_SUCCESS) {
        throw std::system_error(error, cl::opencl_error_category(), "Error setting the second kernel argument");
    }
    size_t offset[2]         = {0, 0};
    size_t global_size[2]    = {width, height};
    cl_event process_wait[2] = {copied_ev, acquired_ev};
    error = ::clEnqueueNDRangeKernel(command_queue, mosaic, 2, offset, global_size, nullptr, 2, process_wait, &ev);
    cl::event filled_ev(ev, ::clReleaseEvent);
    if (error != CL_SUCCESS) {
        throw std::system_error(error, cl::opencl_error_category(), "Error running the mosaic kernel");
    }

    // Release the surface back to vaapi
    cl_event release_wait[1] = {filled_ev};
    context.release_surface(command_queue, 1, &surface, 1, release_wait, &ev);
    cl::event finished_ev(ev, ::clReleaseEvent);
    if (error != CL_SUCCESS) {
        throw std::system_error(error, cl::opencl_error_category(), "Error acquiring the VAAPI surface");
    }

    // Wait for everything to be done
    cl_event finished_wait[1] = {finished_ev};
    clWaitForEvents(1, finished_wait);
}

}  // namespace module::output::compressor::vaapi::cl
