#ifndef MODULE_VISION_REPROJECTION_H
#define MODULE_VISION_REPROJECTION_H

#include <armadillo>
#include <string>
#include <vector>

#include <nuclear>

#define CL_HPP_MINIMUM_OPENCL_VERSION 120
#define CL_HPP_TARGET_OPENCL_VERSION 120
#include <CL/cl2.hpp>

namespace module {
namespace vision {

    class Reprojection : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the Reprojection reactor.
        explicit Reprojection(std::unique_ptr<NUClear::Environment> environment);

    private:
        bool dump_images;
        std::chrono::duration<double, std::milli> avg_fp_ms;
        size_t avg_count;
        cl::Platform platform;
        cl::Device device;
        cl::Context context;
        cl::CommandQueue command_queue;
        cl::Program program;
        bool use_gpu;

        std::function<cl::Event(const cl::EnqueueArgs&,      // The number of workers to spawn etc
                                const cl::Image2D&,          // The input image
                                const cl::Sampler&,          // The input image sampler
                                uint,                        // The format of the image
                                float,                       // The number radians spanned by a single pixel in the
                                                             // spherical image
                                const cl::array<float, 2>&,  // The coordinates of the center of the spherical image
                                float,                       // The focal length of the camera in pixels, for the
                                                             // equirectangular image
                                cl::Image2D&)>               // The output equirectanguler image
            reprojection;

        arma::uvec2 output_dimensions;
        double tan_half_FOV;
    };

}  // namespace vision
}  // namespace module

#endif  // MODULE_VISION_REPROJECTION_H
