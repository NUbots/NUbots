#ifndef MODULE_VISION_CUDA_REPROJECTION_H
#define MODULE_VISION_CUDA_REPROJECTION_H

#include <armadillo>
#include <fstream>
#include <string>
#include <vector>

#include <nuclear>

#include "reprojection.cuh"

namespace module {
namespace vision {
    namespace CUDA {

        class Reprojection : public NUClear::Reactor {

        public:
            /// @brief Called by the powerplant to build and setup the Reprojection reactor.
            explicit Reprojection(std::unique_ptr<NUClear::Environment> environment);

        private:
            bool dump_images;
            bool dump_stats;
            std::ofstream stats_file;
            float avg_fp_ms;
            size_t avg_count;
            int device;

            arma::uvec2 output_dimensions;
            double tan_half_FOV;

            bool cudaCheckError(const cudaError_t& err);
            std::pair<int, int> getCudaComputeCapability(int device);
        };

    }  // namespace CUDA
}  // namespace vision
}  // namespace module

#endif  // MODULE_VISION_CUDA_REPROJECTION_H
