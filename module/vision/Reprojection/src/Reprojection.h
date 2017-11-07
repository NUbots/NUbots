#ifndef MODULE_VISION_REPROJECTION_H
#define MODULE_VISION_REPROJECTION_H

#include <armadillo>
#include <vector>

#include <nuclear>

namespace module {
namespace vision {

    class Reprojection : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the Reprojection reactor.
        explicit Reprojection(std::unique_ptr<NUClear::Environment> environment);

    private:
        arma::uvec2 output_dimensions;
        double tan_half_FOV;
    };

}  // namespace vision
}  // namespace module

#endif  // MODULE_VISION_REPROJECTION_H
