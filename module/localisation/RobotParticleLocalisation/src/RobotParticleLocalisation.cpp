#include "RobotParticleLocalisation.h"

#include "message/input/Sensors.h"
#include "extension/Configuration.h"
#include "message/localisation/FieldObject.h"

#include "message/platform/darwin/DarwinSensors.h"
#include "message/behaviour/Nod.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/time/time.h"


namespace module {
namespace localisation {

    using extension::Configuration;

    using message::input::Sensors;
    using message::localisation::Self;
    using message::platform::darwin::ButtonLeftDown;
    using message::behaviour::Nod;

    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;
    using utility::time::TimeDifferenceSeconds;

    RobotParticleLocalisation::RobotParticleLocalisation(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        last_measurement_update_time = NUClear::clock::now();
        last_time_update_time = NUClear::clock::now();

        on<Configuration>("RobotParticleLocalisation.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file RobotParticleLocalisation.yaml
            filter.model.processNoiseDiagonal = config["process_noise_diagonal"].as<arma::vec>();
            test_state = config["test_state"].as<arma::vec>();

        });

        on<Trigger<Sensors>, Single>().then("Time Update", [this](const Sensors& sensors){
            log("TimeUpdate Start");
            /* Perform time update */
            auto curr_time = NUClear::clock::now();
            double seconds = TimeDifferenceSeconds(curr_time,last_time_update_time);
            last_time_update_time = curr_time;
            filter.timeUpdate(seconds);

            std::vector<arma::vec> test_positions;
            test_positions.push_back(arma::vec{10,10,0});
            test_positions.push_back(arma::vec{2,2,0});

            //Debug filter
            arma::vec measurement = filter.model.predictedObservation(test_state, test_positions[0], sensors);//r,theta,phi
            arma::mat var = arma::diagmat(arma::vec({0.1,0.1,0.01}));
            filter.ambiguousMeasurementUpdate(measurement,var,test_positions,sensors);

            //Emit state
            auto selfs = std::make_unique<std::vector<Self>>();
            selfs->push_back(Self());
            //Todo: get filter state and transform
            arma::vec3 state = filter.get();//filter.getBest();
            selfs->back().locObject.position = Eigen::Vector2d(state[RobotModel::kX],state[RobotModel::kY]);
            selfs->back().heading = Eigen::Vector2d(std::cos(state[RobotModel::kAngle]),std::sin(state[RobotModel::kAngle]));
            // log("measurement", measurement.t());
            log("sensors world", state.t());
            log("deltaT", seconds);

            emit(selfs);
            log("TimeUpdate End");
        });
    }
}
}
