#include "RobotParticleLocalisation.h"

#include "message/input/Sensors.h"
#include "extension/Configuration.h"
#include "message/localisation/FieldObject.h"

#include "message/platform/darwin/DarwinSensors.h"
#include "message/behaviour/Nod.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/math/geometry/Circle.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/time/time.h"

#include "utility/nubugger/NUhelpers.h"

namespace module {
namespace localisation {

    using extension::Configuration;

    using message::input::Sensors;
    using message::localisation::Self;
    using message::platform::darwin::ButtonLeftDown;
    using message::behaviour::Nod;

    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;
    using utility::nubugger::graph;
    using utility::nubugger::drawCircle;
    using utility::math::geometry::Circle;
    using utility::time::TimeDifferenceSeconds;

    RobotParticleLocalisation::RobotParticleLocalisation(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        last_measurement_update_time = NUClear::clock::now();
        last_time_update_time = NUClear::clock::now();

        on<Configuration>("RobotParticleLocalisation.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file RobotParticleLocalisation.yaml
            filter.model.processNoiseDiagonal = config["process_noise_diagonal"].as<arma::vec>();
            filter.model.n_rogues = config["n_rogues"].as<int>();
            filter.model.resetRange = config["reset_range"].as<arma::vec>();
            test_state = config["test_state"].as<arma::vec>();
            int n_particles = config["n_particles"].as<int>();
            draw_particles = config["draw_particles"].as<int>();


            std::vector<arma::vec3> possible_states;
            std::vector<arma::mat33> possible_var;

            possible_states.push_back(test_state);
            //Reflected position
            possible_states.push_back(arma::vec3{test_state[0],-test_state[1],-test_state[2]});

            possible_var.push_back(arma::diagmat(arma::vec3{0.01,0.01,0.001}));
            possible_var.push_back(arma::diagmat(arma::vec3{0.01,0.01,0.001}));

            filter.resetAmbiguous(possible_states, possible_var, n_particles);

        });

        constexpr int PARTICLE_UPDATE_RATE = 1;
        on<Every<PARTICLE_UPDATE_RATE,std::chrono::seconds>,Sync<RobotParticleLocalisation>>().then([this](){
            arma::mat particles = filter.getParticles();
            for(int i =0; i < std::min(draw_particles,int(particles.n_rows)); i++){
                emit(drawCircle("particle"+std::to_string(i),Circle(0.01, particles.submat(i,0,i,1).t()), 0.05, {0, 0, 0},PARTICLE_UPDATE_RATE));
            }
        });

        on<Trigger<Sensors>, Single>().then("Time Update", [this](const Sensors& sensors){
            //First debug particles


            /* Perform time update */
            auto curr_time = NUClear::clock::now();
            double seconds = TimeDifferenceSeconds(curr_time,last_time_update_time);
            last_time_update_time = curr_time;
            filter.timeUpdate(seconds);

            //GOAL TEST
            // std::vector<arma::vec> test_positions_left;
            // //Forward goal
            // test_positions_left.push_back(arma::vec{4.5,1.5,0});
            // //Rear goal
            // test_positions_left.push_back(arma::vec{-4.5,-1.5,0});

            // std::vector<arma::vec> test_positions_right;
            // //Forward goal
            // test_positions_right.push_back(arma::vec{4.5,-1.5,0});
            // //Rear goal
            // test_positions_right.push_back(arma::vec{-4.5,1.5,0});


            // //Debug filter
            // arma::vec measurement_l = filter.model.predictedObservation(test_state, test_positions_left[0], sensors);//r,theta,phi
            // arma::vec measurement_r = filter.model.predictedObservation(test_state, test_positions_right[0], sensors);//r,theta,phi
            // arma::mat var = arma::diagmat(arma::vec({0.1,0.1,0.1}));
            // //Measure both left and right goals
            // float quality = filter.ambiguousMeasurementUpdate(measurement_l,var,test_positions_left,sensors);
            // filter.ambiguousMeasurementUpdate(measurement_r,var,test_positions_right,sensors);
            //GOAL TEST END

            std::vector<arma::vec> test_positions;
            //Forward goal
            test_positions.push_back(arma::vec{4.5,0,0});
            //Rear goal
            test_positions.push_back(arma::vec{-4.5,0,0});

            //Debug filter
            arma::vec3 complement_position({test_state[0],-test_state[1],-test_state[2]});
            arma::vec measurement = filter.model.predictedObservation(test_state, test_positions[1], sensors);//r,theta,phi
            arma::mat var = arma::diagmat(arma::vec({0.1,0.1,0.1}));
            //Measure both left and right goals
            float quality = filter.ambiguousMeasurementUpdate(measurement,var,test_positions,sensors);

            //Emit state
            auto selfs = std::make_unique<std::vector<Self>>();
            selfs->push_back(Self());
            //Todo: get filter state and transform
            // arma::vec3 state = filter.getBest();
            arma::vec3 state = filter.get();
            selfs->back().locObject.position = Eigen::Vector2d(state[RobotModel::kX],state[RobotModel::kY]);
            selfs->back().heading = Eigen::Vector2d(std::cos(state[RobotModel::kAngle]),std::sin(state[RobotModel::kAngle]));
            emit(graph("filter state = ", state[0],state[1],state[2]));
            emit(graph("actual state = ", test_state[0],test_state[1],test_state[2]));
            // log("deltaT", seconds);

            emit(selfs);
        });
    }
}
}
