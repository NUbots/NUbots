#ifndef MODULE_VISION_IGUSVISIONTESTS_H
#define MODULE_VISION_IGUSVISIONTESTS_H

#include <nuclear>

#include "utility/math/vision.h"
#include "utility/support/eigen_armadillo.h"

#include "message/input/CameraParameters.h"
#include "message/input/Image.h"
#include "message/input/Sensors.h"
#include "message/vision/ClassifiedImage.h"

namespace module {
namespace vision {

    class IgusVisionTests : public NUClear::Reactor {

    private:
        float radius          = 0.1;
        arma::vec3 ballCentre = arma::vec3({2, 0, 0});
        float theta_count     = 100;

        arma::vec3 horizon_normal = {0, 0, 1};

        arma::vec3 goal_position  = arma::vec3({2, 1, -0.9});
        arma::vec3 goal_direction = arma::vec3({2, 1, -0.9});

        float lambda      = 0.1;
        arma::vec2 offset = arma::vec2({0, 0});

        arma::vec2 test_point_screen;
        float visual_horizon_height = 0.5;

        std::shared_ptr<const message::input::Image> image;
        std::shared_ptr<const message::input::Sensors> sensors;

    public:
        /// @brief Called by the powerplant to build and setup the IgusVisionTests reactor.
        explicit IgusVisionTests(std::unique_ptr<NUClear::Environment> environment);
        void emitClassifiedImage(const message::input::CameraParameters& cam,
                                 const message::support::FieldDescription& fd);
        std::vector<message::vision::ClassifiedImage::Segment> getGoalSegments(
            const message::input::CameraParameters& cam,
            const message::support::FieldDescription& fd);
    };
}  // namespace vision
}  // namespace module

#endif  // MODULE_VISION_IGUSVISIONTESTS_H
