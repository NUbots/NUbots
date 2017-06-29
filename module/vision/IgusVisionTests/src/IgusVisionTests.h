#ifndef MODULE_VISION_IGUSVISIONTESTS_H
#define MODULE_VISION_IGUSVISIONTESTS_H

#include <nuclear>

#include "utility/support/eigen_armadillo.h"
#include "utility/math/vision.h"

#include "message/vision/VisionObjects.h"
#include "message/vision/ClassifiedImage.h"
#include "message/input/CameraParameters.h"

namespace module {
namespace vision {

    class IgusVisionTests : public NUClear::Reactor {

    private:
        utility::math::vision::RadialCamera::Parameters params;
        arma::uvec2 image_size;

        Eigen::Vector3f ballCentre = Eigen::Vector3f({2,0,0});
        float radius = 0.1;
        float theta_count = 100;

        float lambda = 0.1;
        Eigen::Vector2f offset = Eigen::Vector2f({0,0});

    public:
        /// @brief Called by the powerplant to build and setup the IgusVisionTests reactor.
        explicit IgusVisionTests(std::unique_ptr<NUClear::Environment> environment);
        void emitClassifiedImage();
    };

}
}

#endif  // MODULE_VISION_IGUSVISIONTESTS_H
