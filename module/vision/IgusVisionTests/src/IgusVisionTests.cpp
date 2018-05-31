#include "IgusVisionTests.h"

#include "extension/Configuration.h"
#include "message/support/FieldDescription.h"
#include "utility/nusight/NUhelpers.h"
#include "utility/support/yaml_armadillo.h"

namespace module {
namespace vision {

    using extension::Configuration;
    using message::input::CameraParameters;
    using message::input::Image;
    using message::input::Sensors;
    using message::support::FieldDescription;
    using message::vision::ClassifiedImage;
    using utility::math::vision::getImageFromCam;
    using utility::nusight::drawVisionLines;

    IgusVisionTests::IgusVisionTests(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("IgusVisionTests.yaml").then([this](const Configuration& config) {
            // Use configuration here from file IgusVisionTests.yaml

            // arma::fvec3 ballCentreTemp = config["ballCentre"].as<arma::fvec>();
            // ballCentre = convert<float,3>(ballCentreTemp);
            ballCentre = config["ballCentre"].as<arma::vec>();
            radius     = config["radius"].as<float>();

            horizon_normal = config["horizon_normal"].as<arma::vec>();

            theta_count = config["theta_count"].as<float>();

            test_point_screen = config["test_point_screen"].as<arma::vec>();

            goal_position  = config["goal_position"].as<arma::vec>();
            goal_direction = arma::normalise(config["goal_direction"].as<arma::vec>());

            visual_horizon_height = config["visual_horizon_height"].as<float>();
        });

        on<Trigger<std::vector<message::vision::Ball>>>().then([this](const std::vector<message::vision::Ball>& balls) {
            for (auto& ball : balls) {
                for (auto& m : ball.measurements) {
                    arma::vec3 measuredPos = convert<double, 3>(m.rBCc);
                    log("Ball actual pos (x,y,z):  ", ballCentre.t());
                    log("Ball measured pos (x,y,z):", measuredPos.t());
                    log("Ball detector error =     ", (measuredPos - ballCentre).t());
                    log("Ball norm error =         ", arma::norm(measuredPos - ballCentre));
                    // for(auto& edgePts : ball.edgePoints){
                    //     log("Edge pts:", edgePts);
                    // }
                }
            }
        });
        on<Trigger<std::vector<message::vision::Goal>>>().then([this](const std::vector<message::vision::Goal>& goals) {
            log("Goals: ", goals.size());
            for (auto& goal : goals) {
                for (auto& m : goal.measurement) {
                    if (m.type != message::vision::Goal::MeasurementType::CENTRE) continue;
                    arma::vec3 measuredPos = convert<double, 3>(m.position);
                    log("Goal actual pos (x,y,z):  ", goal_position.t());
                    log("Goal measured pos (x,y,z):", measuredPos.t());
                    log("Goal detector error =     ", (measuredPos - goal_position).t());
                    log("Goal norm error =         ", arma::norm(measuredPos - goal_position));
                    // for(auto& edgePts : ball.edgePoints){
                    //     log("Edge pts:", edgePts);
                    // }
                }
            }
        });


        on<Every<30, Per<std::chrono::seconds>>,
           With<CameraParameters>,
           Optional<With<Image>>,
           Optional<With<Sensors>>,
           With<FieldDescription>>()
            .then([this](const CameraParameters& cam,
                         std::shared_ptr<const Image> inputImage,
                         std::shared_ptr<const Sensors> inputSensors,
                         const FieldDescription& fd) {
                image   = inputImage;
                sensors = inputSensors;
                if (image && sensors) {
                    emitClassifiedImage(cam, fd);
                }
                // arma::vec3 test_point_cam     = utility::math::vision::getCamFromScreen(test_point_screen, cam);
                // arma::vec2 test_point_screen2 = utility::math::vision::projectCamSpaceToScreen(test_point_cam,
                // cam); arma::vec3 test_point_cam2    = utility::math::vision::getCamFromScreen(test_point_screen2,
                // cam); std::cout << "test_point_screen" << test_point_screen.t() << std::endl; std::cout <<
                // "test_point_cam" << test_point_cam.t() << std::endl; std::cout << "test_point_screen2" <<
                // test_point_screen2.t() << std::endl; std::cout << "test_point_cam2" << test_point_cam2.t() <<
                // std::endl;
            });
    }

    void IgusVisionTests::emitClassifiedImage(const CameraParameters& cam, const FieldDescription& fd) {
        // Basis of circle
        arma::vec3 p = ballCentre;
        arma::vec3 q = arma::normalise(arma::vec3({p[1], -p[0], 0}));
        arma::vec3 r = arma::normalise(arma::cross(q, p));

        // Theta parameter
        float theta = 0;
        // Step 10 times if theta_count is zero
        float theta_step = theta_count != 0 ? 2 * M_PI / theta_count : 10;

        std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>> lines;
        // Points on screen
        std::vector<Eigen::Matrix<int, 2, 1, 2>> imagePoints;
        // Generate visible points
        while (theta < 2 * M_PI) {
            // Generate (approximate) circle of visible points
            arma::vec3 P = p + radius * (q * cos(theta) + r * sin(theta));
            // Project to screen
            // const arma::fvec3& point, const Parameters& params = Parameters()
            arma::vec2 pixel = utility::math::vision::projectCamSpaceToScreen(P, cam);
            // Screen point referenced from screen centre
            arma::vec2 screenPoint = arma::vec2({pixel[0], pixel[1]});
            // Convert to point referenced from top left
            arma::ivec2 imagePoint = utility::math::vision::screenToImage(
                screenPoint, arma::uvec2({uint(image->dimensions[0]), uint(image->dimensions[1])}));
            imagePoints.push_back(convert<int, 2>(imagePoint));
            lines.push_back(std::make_pair(imagePoints.back(), imagePoints.back() + Eigen::Vector2i(1, 1)));
            // Increment theta
            // std::cout << "theta: " << theta << " rad, " << theta * (180/M_PI) << " deg:" <<
            //     ", \n\tpoint: (" << P[0] << "," << P[1] << "," << P[2] << ")" <<
            //     ", \n\tpixel: (" << pixel[0] << "," << pixel[1] << ")" <<
            //     ", \n\tscreenPoint: (" << screenPoint[0] << "," << screenPoint[1] << ")" <<
            //     ", \n\timagePoint: (" << imagePoint[0] << "," << imagePoint[1] << ")\n" << std::endl;
            theta += theta_step;
        }


        auto classifiedImage                = std::make_unique<ClassifiedImage>();
        classifiedImage->ballPoints         = imagePoints;
        classifiedImage->ballSeedPoints[0]  = imagePoints;
        classifiedImage->ballSeedPoints[1]  = imagePoints;
        classifiedImage->ballSeedPoints[2]  = imagePoints;
        classifiedImage->image              = const_cast<Image*>(image.get())->shared_from_this();
        classifiedImage->sensors            = const_cast<Sensors*>(sensors.get())->shared_from_this();
        classifiedImage->horizon_normal     = convert<double, 3>(horizon_normal);
        classifiedImage->horizontalSegments = getGoalSegments(cam, fd);
        // classifiedImage->horizon.distance = 200;
        classifiedImage->dimensions = image->dimensions;
        classifiedImage->visualHorizon.push_back(
            Eigen::Vector2i(0, visual_horizon_height * (image->dimensions[1] - 1)));
        classifiedImage->visualHorizon.push_back(
            Eigen::Vector2i(image->dimensions[0] - 1, visual_horizon_height * (image->dimensions[1] - 1)));

        emit(drawVisionLines(lines, Eigen::Vector4d({1, 1, 1, 1})));
        emit(classifiedImage);
    }


    std::vector<message::vision::ClassifiedImage::Segment> IgusVisionTests::getGoalSegments(
        const message::input::CameraParameters& cam,
        const message::support::FieldDescription& fd) {
        std::vector<message::vision::ClassifiedImage::Segment> segments;

        int N             = theta_count;
        float max_height  = fd.dimensions.goal_crossbar_height;
        float step_height = max_height / N;
        float width       = fd.dimensions.goalpost_width;

        for (float height = 0; height < max_height; height += step_height) {


            arma::vec3 center = goal_position + height * goal_direction;
            arma::vec3 left   = center + (width / 2) * arma::vec3({0, 1, 0});
            arma::vec3 right  = center + (width / 2) * arma::vec3({0, -1, 0});

            segments.push_back(message::vision::ClassifiedImage::Segment());
            segments.back().segmentClass = message::vision::ClassifiedImage::SegmentClass::GOAL;
            segments.back().start        = convert<int, 2>(getImageFromCam(left, cam));
            segments.back().end          = convert<int, 2>(getImageFromCam(right, cam));
            segments.back().midpoint     = convert<int, 2>(getImageFromCam(center, cam));
            segments.back().next         = 0;
            segments.back().previous     = 0;
            segments.back().subsample    = 1;
            segments.back().length       = arma::norm(left - right);
        }
        return segments;
    }
}  // namespace vision
}  // namespace module
