#ifndef MODULE_VISION_IGUSVISIONTESTS_H
#define MODULE_VISION_IGUSVISIONTESTS_H

#include <nuclear>

#include "utility/math/vision.h"
#include "utility/support/eigen_armadillo.h"

#include "message/input/CameraParameters.h"
#include "message/input/Image.h"
#include "message/input/Sensors.h"
#include "message/vision/ClassifiedImage.h"
#include "message/vision/VisionObjects.h"

namespace module {
namespace vision {

    class IgusVisionTests : public NUClear::Reactor {

    private:
        float radius          = 0.1;
        arma::vec3 ballCentre = arma::vec3({2, 0, 0});
        float theta_count     = 100;

        float lambda      = 0.1;
        arma::vec2 offset = arma::vec2({0, 0});

        arma::vec2 test_point_screen;

        std::shared_ptr<const message::input::Image> image;
        std::shared_ptr<const message::input::Sensors> sensors;

    public:
        /// @brief Called by the powerplant to build and setup the IgusVisionTests reactor.
        explicit IgusVisionTests(std::unique_ptr<NUClear::Environment> environment);
        void emitClassifiedImage(const message::input::CameraParameters& cam);
    };
}
}

#endif  // MODULE_VISION_IGUSVISIONTESTS_H
