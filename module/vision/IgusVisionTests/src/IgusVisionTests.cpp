#include "IgusVisionTests.h"

#include "utility/support/yaml_armadillo.h"
#include "utility/nubugger/NUhelpers.h"
#include "extension/Configuration.h"

namespace module {
namespace vision {

    using message::input::Image;
    using message::input::Sensors;
    using message::vision::ClassifiedImage;
    using utility::nubugger::drawVisionLines;
    using extension::Configuration;
    using message::input::CameraParameters;

    IgusVisionTests::IgusVisionTests(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("IgusVisionTests.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file IgusVisionTests.yaml

            // arma::fvec3 ballCentreTemp = config["ballCentre"].as<arma::fvec>();
            // ballCentre = convert<float,3>(ballCentreTemp);
            ballCentre = config["ballCentre"].as<arma::vec>();
            radius = config["radius"].as<float>();

            theta_count = config["theta_count"].as<float>();

        });

        on<Trigger<std::vector<message::vision::Ball>>>().then([this] (const std::vector<message::vision::Ball>& balls) {
            for(auto& ball : balls){
                log("Ball detected by vision, pos:", ball.position.transpose());
                for(auto& edgePts : ball.edgePoints){
                    log("Edge pts:", edgePts);
                }
            }
        });

        on<Every<30, Per<std::chrono::seconds>>,
            With<CameraParameters>,
            Optional<With <Image>>,
            Optional<With <Sensors>>
            >().then([this](
                const CameraParameters& cam,
                std::shared_ptr<const Image> inputImage,
                std::shared_ptr<const Sensors> inputSensors) {
            image = inputImage;
            sensors = inputSensors;
            if(image && sensors){
                emitClassifiedImage(cam);
            }
        });
    }

    void IgusVisionTests::emitClassifiedImage(const CameraParameters& cam){
        //Basis of circle
        arma::vec3 p = ballCentre;
        arma::vec3 q = arma::normalise(arma::vec3({p[1], -p[0], 0}));
        arma::vec3 r = arma::normalise(arma::cross(q,p));

        //Theta parameter
        float theta = 0;
        //Step 10 times if theta_count is zero
        float theta_step = theta_count != 0 ? 2 * M_PI / theta_count : 10;

        std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>> lines;
        //Points on screen
        std::vector<Eigen::Matrix<int,2,1,2>> imagePoints;
        //Generate visible points
        while(theta < 2 * M_PI){
            //Generate (approximate) circle of visible points
            arma::vec3 P = p + radius * (q * cos(theta) + r * sin(theta));
            //Project to screen
            //const arma::fvec3& point, const Parameters& params = Parameters()
            arma::vec2 pixel = utility::math::vision::projectCamSpaceToScreen(P,cam);
            //Screen point referenced from screen centre
            arma::vec2 screenPoint = arma::vec2({pixel[0],pixel[1]});
            //Convert to point referenced from top left
            arma::ivec2 imagePoint = utility::math::vision::screenToImage(screenPoint,arma::uvec2({uint(image->dimensions[0]), uint(image->dimensions[1])}));
            imagePoints.push_back(convert<int,2>(imagePoint));
            lines.push_back(std::make_pair(imagePoints.back(),imagePoints.back()+Eigen::Vector2i(1,1)));
            //Increment theta
            // std::cout << "theta: " << theta << " rad, " << theta * (180/M_PI) << " deg:" <<
            //     ", \n\tpoint: (" << P[0] << "," << P[1] << "," << P[2] << ")" <<
            //     ", \n\tpixel: (" << pixel[0] << "," << pixel[1] << ")" <<
            //     ", \n\tscreenPoint: (" << screenPoint[0] << "," << screenPoint[1] << ")" <<
            //     ", \n\timagePoint: (" << imagePoint[0] << "," << imagePoint[1] << ")\n" << std::endl;
            theta += theta_step;
        }


        auto classifiedImage = std::make_unique<ClassifiedImage>();
        classifiedImage->ballPoints = imagePoints;
        classifiedImage->ballSeedPoints[0] = imagePoints;
        classifiedImage->ballSeedPoints[1] = imagePoints;
        classifiedImage->ballSeedPoints[2] = imagePoints;
        classifiedImage->image = const_cast<Image*>(image.get())->shared_from_this();
        classifiedImage->sensors = const_cast<Sensors*>(sensors.get())->shared_from_this();
        classifiedImage->horizon.normal = Eigen::Vector2d(0,-1);
        classifiedImage->horizon.distance = 200;
        classifiedImage->dimensions = image->dimensions;

        emit(drawVisionLines(lines, Eigen::Vector4d({1,1,1,1})));
        emit(classifiedImage);
    }
}
}
