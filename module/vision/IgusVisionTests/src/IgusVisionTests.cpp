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
        std::vector<Eigen::Vector2i> screenPoints;
        //Generate visible points
        while(theta < 2 * M_PI){
            Eigen::Vector3f P = p + q * cos(theta) + r * sin(theta);




            theta += theta_step;
        }



        log(P);
    }
}
}
