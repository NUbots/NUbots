#include "RobotParticleLocalisation.h"
#include "extension/Configuration.h"
#include "message/input/Sensors.h"
#include "message/localisation/FieldObject.h"
#include "stdio.h"

#include "message/behaviour/Nod.h"
#include "message/platform/darwin/DarwinSensors.h"
#include "utility/math/geometry/Circle.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/support/eigen_armadillo.h"
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

    using message::support::FieldDescription;
    using message::vision::Goal;

    RobotParticleLocalisation::RobotParticleLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        last_measurement_update_time = NUClear::clock::now();
        last_time_update_time        = NUClear::clock::now();

        on<Configuration>("RobotParticleLocalisation.yaml").then([this](const Configuration& config) {
            // Use configuration here from file RobotParticleLocalisation.yaml
            filter.model.processNoiseDiagonal = config["process_noise_diagonal"].as<arma::vec>();
            filter.model.n_rogues             = config["n_rogues"].as<int>();
            filter.model.resetRange           = config["reset_range"].as<arma::vec>();
            int n_particles                   = config["n_particles"].as<int>();
            draw_particles                    = config["draw_particles"].as<int>();

            arma::vec3 start_state    = config["start_state"].as<arma::vec>();
            arma::vec3 start_variance = config["start_variance"].as<arma::vec>();

            std::vector<arma::vec3> possible_states;
            std::vector<arma::mat33> possible_var;

            possible_states.push_back(start_state);
            // Reflected position
            possible_states.push_back(arma::vec3{start_state[0], -start_state[1], -start_state[2]});

            possible_var.push_back(arma::diagmat(start_variance));
            possible_var.push_back(arma::diagmat(start_variance));

            filter.resetAmbiguous(possible_states, possible_var, n_particles);

        });

        on<Every<PARTICLE_UPDATE_FREQUENCY, Per<std::chrono::seconds>>, Sync<RobotParticleLocalisation>>().then(
            "Particle Debug", [this]() {
                arma::mat particles = filter.getParticles();
                for (int i = 0; i < std::min(draw_particles, int(particles.n_rows)); i++) {
                    emit(drawCircle("particle" + std::to_string(i),
                                    Circle(0.01, particles.submat(i, 0, i, 1).t()),
                                    0.05,
                                    {0, 0, 0},
                                    PARTICLE_UPDATE_FREQUENCY));
                }
            });

        on<Every<TIME_UPDATE_FREQUENCY, Per<std::chrono::seconds>>, Sync<RobotParticleLocalisation>>().then(
            "Time Update", [this]() {
                /* Perform time update */
                auto curr_time        = NUClear::clock::now();
                double seconds        = TimeDifferenceSeconds(curr_time, last_time_update_time);
                last_time_update_time = curr_time;

                filter.timeUpdate(seconds);

                // Emit state
                auto selfs = std::make_unique<std::vector<Self>>();
                selfs->push_back(Self());
                // emit(graph("Self",Self.locObject.position[0], Self.locObject.position[1],))

                // Get filter state and transform
                arma::vec3 state                 = filter.get();
                selfs->back().locObject.position = Eigen::Vector2d(state[RobotModel::kX], state[RobotModel::kY]);
                selfs->back().heading =
                    Eigen::Vector2d(std::cos(state[RobotModel::kAngle]), std::sin(state[RobotModel::kAngle]));
                selfs->back().covariance = convert<double, 3, 3>(filter.getCovariance());
                emit(graph("robot filter state = ", state[0], state[1], state[2]));

                auto self = std::make_unique<Self>((*selfs)[0]);
                emit(self);
                emit(selfs);

            });

        on<Trigger<std::vector<Goal>>, With<FieldDescription>, Sync<RobotParticleLocalisation>>().then(
            "Measurement Update", [this](const std::vector<Goal>& goals, const FieldDescription& fd) {

                if (!goals.empty()) {
                    // First debug particles
                    const auto& sensors = *goals[0].visObject.sensors;
                    /* Perform time update */
                    auto curr_time        = NUClear::clock::now();
                    double seconds        = TimeDifferenceSeconds(curr_time, last_time_update_time);
                    last_time_update_time = curr_time;

                    filter.timeUpdate(seconds);

                    for (auto goal : goals) {

                        // Check side and team
                        std::vector<arma::vec> poss = getPossibleFieldPositions(goal, fd);

                        for (auto& m : goal.measurement) {
                            if (m.type == Goal::MeasurementType::TOP_NORMAL) continue;
                            if (m.type == Goal::MeasurementType::BASE_NORMAL) continue;
                            if (m.type == Goal::MeasurementType::UNKNOWN_MEASUREMENT) continue;
                            // Measure objects
                            if (m.type == Goal::MeasurementType::CENTRE) {
                                // log(goal.side == Goal::Side::LEFT
                                //         ? (" LEFT GOAL ")
                                //         : (goal.side == Goal::Side::RIGHT ? " RIGHT GOAL " : " UNKNOWN GOAL "),
                                //     convert<double, 3>(m.position).t());
                                filter.ambiguousMeasurementUpdate(convert<double, 3>(m.position),
                                                                  convert<double, 3, 3>(m.covariance),
                                                                  poss,
                                                                  sensors,
                                                                  m.type,
                                                                  fd);
                            }
                            else {
                                // filter.ambiguousMeasurementUpdate(convert<double, 2>(m.normalAngles),
                                //                                   convert<double, 2, 2>(m.normAngCov),
                                //                                   poss,
                                //                                   sensors,
                                //                                   m.type,
                                //                                   fd);
                            }
                        }
                    }
                }
            });
    }

    std::vector<arma::vec> RobotParticleLocalisation::getPossibleFieldPositions(
        const message::vision::Goal& goal,
        const message::support::FieldDescription& fd) const {
        std::vector<arma::vec> possibilities;

        bool left  = (goal.side != Goal::Side::RIGHT);
        bool right = (goal.side != Goal::Side::LEFT);
        bool own   = (goal.team != Goal::Team::OPPONENT);
        bool opp   = (goal.team != Goal::Team::OWN);

        if (own && left) {
            possibilities.push_back(arma::vec3({fd.goalpost_own_l[0], fd.goalpost_own_l[1], 0}));
            // std::cout << __FILE__ << ", " << __LINE__ << (" Goal possibility own  left") << std::endl;
        }
        if (own && right) {
            possibilities.push_back(arma::vec3({fd.goalpost_own_r[0], fd.goalpost_own_r[1], 0}));
            // std::cout << __FILE__ << ", " << __LINE__ << (" Goal possibility own  right") << std::endl;
        }
        if (opp && left) {
            possibilities.push_back(arma::vec3({fd.goalpost_opp_l[0], fd.goalpost_opp_l[1], 0}));
            // std::cout << __FILE__ << ", " << __LINE__ << (" Goal possibility opp  left") << std::endl;
        }
        if (opp && right) {
            possibilities.push_back(arma::vec3({fd.goalpost_opp_r[0], fd.goalpost_opp_r[1], 0}));
            // std::cout << __FILE__ << ", " << __LINE__ << (" Goal possibility opp  right") << std::endl;
        }

        return possibilities;
    }
}  // namespace localisation
}  // namespace module
