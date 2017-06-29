#include "IgusVisionTests.h"

#include "extension/Configuration.h"
#include "utility/support/eigen_armadillo.h"

namespace module {
namespace vision {

    using extension::Configuration;

    IgusVisionTests::IgusVisionTests(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("IgusVisionTests.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file IgusVisionTests.yaml
            ballCentre = config["ballCentre"].as<Eigen::Vector3f>();
            radius = config["radius"].as<float>();

            theta_count = config["theta_count"].as<float>();

            params.lambda = config["lambda"].as<float>();
            params.offset = config["offset"].as<arma::fvec>();

            image_size = config["image_size"].as<arma::uvec>();
            emitClassifiedImage();
        });
    }

    void IgusVisionTests::emitClassifiedImage(){
        //Basis of circle
        Eigen::Vector3f p = ballCentre;
        Eigen::Vector3f q = Eigen::Vector3f({p[1], -p[0], 0});
        Eigen::Vector3f r = q.cross(p);

        //Theta parameter
        float theta = 0;
        //Step 10 times if theta_count is zero
        float theta_step = theta_count != 0 ? 2 * M_PI / theta_count : 10;

        //Points on screen
        std::vector<Eigen::Vector2i> imagePoints;
        //Generate visible points
        while(theta < 2 * M_PI){
            //Generate (approximate) circle of visible points
            Eigen::Vector3f P = p + q * cos(theta) + r * sin(theta);
            //Project to screen
            arma::fvec2 pixel = math::utility::vision::RadialCamera::pointToPixel(P,params);
            //Screen point referenced from screen centre
            arma::ivec2 screenPoint = arma::ivec2({int(pixel[0]),int(pixel[1])});
            //Convert to point referenced from top left
            arma::ivec2 imagePoint = math::utility::vision::screenToImage(screen,image_size);
            imagePoints.push_back(imagePoint);
            //Increment theta
            theta += theta_step;
        }

        for(auto& p : imagePoints){
            log(p.t());
        }
    }
}
}
