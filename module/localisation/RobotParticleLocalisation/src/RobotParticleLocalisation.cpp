#include "RobotParticleLocalisation.h"

#include "message/input/Sensors.h"
#include "extension/Configuration.h"
#include "message/localisation/FieldObject.h"

#include "message/platform/darwin/DarwinSensors.h"
#include "message/behaviour/Nod.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/support/yaml_armadillo.h"

namespace module {
namespace localisation {

    using extension::Configuration;

    using message::input::Sensors;
    using message::localisation::Self;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;
    using message::platform::darwin::ButtonLeftDown;
    using message::behaviour::Nod;
    RobotParticleLocalisation::RobotParticleLocalisation(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        last_measurement_update_time = NUClear::clock::now();
        last_time_update_time = NUClear::clock::now();

        on<Configuration>("RobotParticleLocalisation.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file RobotParticleLocalisation.yaml
            filter.model.processNoiseDiagonal = config["process_noise_diagonal"].as<arma::vec>();
        });

        on<Trigger<Sensors>, Single>().then("Time Update", [this](const Sensors& sensors){

            /* Perform time update */
            auto curr_time = NUClear::clock::now();
            double seconds = TimeDifferenceSeconds(curr_time,last_time_update_time);
            last_time_update_time = curr_time;
            filter.timeUpdate(seconds);

            std::vector<arma::vec> test_positions;
            test_positions.push_back(arma::vec{10,10});
            test_positions.push_back(arma::vec{2,2});
            arma::vec measurement({1,0,0});//r,theta,phi
            arma::mat var = arma::diagmat(arma::vec({1,1,1}));
            filter.ambiguousMeasurementUpdate(measurement,var,test_positions,sensors);


            auto selfs = std::make_unique<std::vector<Self>>();
            selfs->push_back(Self());
            //Todo: get filter state and transform
            selfs->back().locObject.position = convert<double, 2, 1>(state.xy());
            selfs->back().heading = Eigen::Vector2d(std::cos(state.angle()),std::sin(state.angle()));
            // log("sensors world", state.t());
            // log("offset", localisationOffset.t());
            // log("world", Twr.t());
            emit(selfs);
        });
    }
}
}
