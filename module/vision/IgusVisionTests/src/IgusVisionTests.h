#ifndef MODULE_VISION_IGUSVISIONTESTS_H
#define MODULE_VISION_IGUSVISIONTESTS_H

#include <nuclear>
#include "utility/math/vision.h"

namespace module {
namespace vision {

    class IgusVisionTests : public NUClear::Reactor {

    private:
        utility::math::vision::RadialCamera::Parameters params;
        arma::uvec2 image_size;

        Eigen::Vector3f ballCentre = Eigen::Vector3f({2,0,0});
        double radius = 0.1;
    public:
        /// @brief Called by the powerplant to build and setup the IgusVisionTests reactor.
        explicit IgusVisionTests(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_VISION_IGUSVISIONTESTS_H
