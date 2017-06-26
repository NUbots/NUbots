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

            emitClassifiedImage();
        });
    }

    void IgusVisionTests::emitClassifiedImage(){
        Eigen::Vector3f q = Eigen::Vector3f({p[1], -p[0], 0}), tempP;
        float theta = 0;
        std::vector<Eigen::Vector3f> P;
        while(theta < 6.28318530718){
            tempP = ballCentre + q*cos(theta) + radius*sin(theta);
            P.push_back(tempP);
            theta += 0.0174533;
        }

        log(P);
    }
}
}
