#include "IgusVisionTests.h"

#include "utility/support/yaml_armadillo.h"
#include "extension/Configuration.h"

namespace module {
namespace vision {

    using message::input::Image;
    using message::input::Sensors;
    using message::vision::ClassifiedImage;
    using extension::Configuration;

    IgusVisionTests::IgusVisionTests(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("IgusVisionTests.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file IgusVisionTests.yaml

            // arma::fvec3 ballCentreTemp = config["ballCentre"].as<arma::fvec>();
            // ballCentre = convert<float,3>(ballCentreTemp);
            ballCentre = config["ballCentre"].as<arma::vec>();
            radius = config["radius"].as<float>();

            theta_count = config["theta_count"].as<float>();

            params.lambda = config["lambda"].as<float>();
            params.offset = config["offset"].as<arma::vec>();
        });

        on<Trigger<std::vector<message::vision::Ball>>>().then([this] (const std::vector<message::vision::Ball>& balls) {
            for(auto& ball : balls){
                log("Ball pos:", ball.position);
                for(auto& edgePts : ball.edgePoints){
                    log("Edge pts:", edgePts);
                }
            }
        });

        on<Every<30, Per<std::chrono::seconds>>,
            Optional<With <Image>>,
            Optional<With <Sensors>>
            >().then([this](std::shared_ptr<const Image> inputImage,
                std::shared_ptr<const Sensors> inputSensors) {
            image = inputImage;
            sensors = inputSensors;
            if(image && sensors){
                emitClassifiedImage();
            }
        });
    }

    void IgusVisionTests::emitClassifiedImage(){
        //Basis of circle
        arma::vec3 p = ballCentre;
        arma::vec3 q = arma::vec3({p[1], -p[0], 0});
        arma::vec3 r = arma::cross(q,p);

        //Theta parameter
        float theta = 0;
        //Step 10 times if theta_count is zero
        float theta_step = theta_count != 0 ? 2 * M_PI / theta_count : 10;

        //Points on screen
        std::vector<Eigen::Matrix<int,2,1,2>> imagePoints;
        //Generate visible points
        while(theta < 2 * M_PI){
            //Generate (approximate) circle of visible points
            arma::vec3 P = p + q * cos(theta) + r * sin(theta);
            //Project to screen
            //const arma::fvec3& point, const Parameters& params = Parameters()
            arma::vec2 pixel = utility::math::vision::RadialCamera::pointToPixel(P,params);
            //Screen point referenced from screen centre
            arma::vec2 screenPoint = arma::vec2({pixel[0],pixel[1]});
            //Convert to point referenced from top left
            arma::ivec2 imagePoint = utility::math::vision::screenToImage(screenPoint,arma::uvec2({uint(image->dimensions[0]), uint(image->dimensions[1])}));
            imagePoints.push_back(convert<int,2>(imagePoint));
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
        classifiedImage->image = const_cast<Image*>(image.get())->shared_from_this();
        classifiedImage->sensors = const_cast<Sensors*>(sensors.get())->shared_from_this();
        classifiedImage->horizon.normal = Eigen::Vector2d(0,-1);
        classifiedImage->horizon.distance = 200;
        classifiedImage->dimensions = image->dimensions;

        emit(classifiedImage);
    }
}
}
