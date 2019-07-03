#include "RobotParticleLocalisation.h"

#include "extension/Configuration.h"
#include "message/input/Sensors.h"
#include "message/localisation/Field.h"
#include "message/localisation/ResetRobotHypotheses.h"
#include "message/vision/Goal.h"
#include "utility/localisation/transform.h"

#include "utility/math/geometry/Circle.h"
#include "utility/nusight/NUhelpers.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/time/time.h"

namespace module {
namespace localisation {

    using extension::Configuration;

    using message::input::Sensors;
    using message::localisation::Field;
    using message::localisation::ResetRobotHypotheses;
    using message::support::FieldDescription;
    using VisionGoal  = message::vision::Goal;
    using VisionGoals = message::vision::Goals;

    using utility::localisation::transform3DToFieldState;
    using utility::math::geometry::Circle;
    using utility::math::matrix::Rotation2D;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;
    using utility::nusight::drawCircle;
    using utility::nusight::graph;
    using utility::time::TimeDifferenceSeconds;

    RobotParticleLocalisation::RobotParticleLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        last_measurement_update_time = NUClear::clock::now();
        last_time_update_time        = NUClear::clock::now();

        on<Every<PARTICLE_UPDATE_FREQUENCY, Per<std::chrono::seconds>>, Sync<RobotParticleLocalisation>>().then(
            "Particle Debug", [this]() {
                arma::mat particles = filter.getParticles();
                for (int i = 0; i < std::min(draw_particles, int(particles.n_cols)); i++) {
                    emit(drawCircle("particle" + std::to_string(i),
                                    Circle(0.01, particles.submat(0, i, 1, i)),
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

                // Get filter state and transform
                arma::vec3 state = filter.get();
                emit(graph("robot filter state = ", state[0], state[1], state[2]));

                // Emit state
                auto field = std::make_unique<Field>();
                field->position =
                    Eigen::Vector3d(state[RobotModel::kX], state[RobotModel::kY], state[RobotModel::kAngle]);
                field->covariance = convert(filter.getCovariance());

                emit(std::make_unique<std::vector<Field>>(1, *field));
                emit(field);
            });

        on<Trigger<VisionGoals>, With<FieldDescription>, Sync<RobotParticleLocalisation>>().then(
            "Measurement Update", [this](const VisionGoals& goals, const FieldDescription& fd) {
                if (!goals.goals.empty()) {
                    /* Perform time update */
                    auto curr_time        = NUClear::clock::now();
                    double seconds        = TimeDifferenceSeconds(curr_time, last_time_update_time);
                    last_time_update_time = curr_time;

                    filter.timeUpdate(seconds);

                    for (auto goal : goals.goals) {

                        // Check side and team
                        std::vector<arma::vec> poss = getPossibleFieldPositions(goal, fd);

                        for (auto& m : goal.measurements) {
                            if (m.type == VisionGoal::MeasurementType::CENTRE) {
                                filter.ambiguousMeasurementUpdate(arma::conv_to<arma::vec>::from(convert(m.position)),
                                                                  arma::conv_to<arma::mat>::from(convert(m.covariance)),
                                                                  poss,
                                                                  convert(goals.Hcw),
                                                                  m.type,
                                                                  fd);
                            }
                        }
                    }
                }
            });

        on<Trigger<ResetRobotHypotheses>, With<Sensors>, Sync<RobotParticleLocalisation>>().then(
            "Reset Robot Hypotheses", [this](const ResetRobotHypotheses& locReset, const Sensors& sensors) {
                Transform3D Hfw;
                const Transform3D& Htw = convert(sensors.Htw);
                std::vector<arma::vec3> states;
                std::vector<arma::mat33> cov;

                for (auto& s : locReset.hypotheses) {
                    Transform3D Hft;
                    arma::vec3 rTFf   = {s.position[0], s.position[1], 0};
                    Hft.translation() = rTFf;
                    Hft.rotateZ(s.heading);
                    Hfw = Hft * Htw;
                    states.push_back(transform3DToFieldState(Hfw));

                    Rotation2D Hfw_xy     = Hfw.projectTo2D(arma::vec3({0, 0, 1}), arma::vec3({1, 0, 0})).rotation();
                    arma::mat22 pos_cov   = Hfw_xy * convert(s.position_cov) * Hfw_xy.t();
                    arma::mat33 state_cov = arma::eye(3, 3);
                    state_cov.submat(0, 0, 1, 1) = pos_cov;
                    state_cov(2, 2)              = s.heading_var;
                    cov.push_back(state_cov);
                }
                filter.resetAmbiguous(states, cov, n_particles);
            });

        on<Configuration>("RobotParticleLocalisation.yaml").then([this](const Configuration& config) {
            // Use configuration here from file RobotParticleLocalisation.yaml
            filter.model.processNoiseDiagonal = config["process_noise_diagonal"].as<arma::vec>();
            filter.model.n_rogues             = config["n_rogues"].as<int>();
            filter.model.resetRange           = config["reset_range"].as<arma::vec>();
            n_particles                       = config["n_particles"].as<int>();
            draw_particles                    = config["draw_particles"].as<int>();

            arma::vec3 start_state    = config["start_state"].as<arma::vec>();
            arma::vec3 start_variance = config["start_variance"].as<arma::vec>();

            auto reset = std::make_unique<ResetRobotHypotheses>();
            ResetRobotHypotheses::Self leftSide;
            // Start on goal line
            leftSide.position     = Eigen::Vector2d(start_state[0], start_state[1]);
            leftSide.position_cov = Eigen::Vector2d::Constant(0.5).asDiagonal();
            leftSide.heading      = start_state[2];
            leftSide.heading_var  = 0.005;

            reset->hypotheses.push_back(leftSide);
            ResetRobotHypotheses::Self rightSide;
            // Start on goal line
            rightSide.position     = Eigen::Vector2d(start_state[0], -start_state[1]);
            rightSide.position_cov = Eigen::Vector2d::Constant(0.5).asDiagonal();
            rightSide.heading      = -start_state[2];
            rightSide.heading_var  = 0.005;

            reset->hypotheses.push_back(rightSide);
            emit<Scope::DELAY>(reset, std::chrono::seconds(1));
        });
    }

    std::vector<arma::vec> RobotParticleLocalisation::getPossibleFieldPositions(
        const VisionGoal& goal,
        const message::support::FieldDescription& fd) const {
        std::vector<arma::vec> possibilities;

        bool left  = (goal.side != VisionGoal::Side::RIGHT);
        bool right = (goal.side != VisionGoal::Side::LEFT);
        bool own   = (goal.team != VisionGoal::Team::OPPONENT);
        bool opp   = (goal.team != VisionGoal::Team::OWN);

        if (own && left) {
            possibilities.push_back(arma::vec3({fd.goalpost_own_l[0], fd.goalpost_own_l[1], 0}));
        }
        if (own && right) {
            possibilities.push_back(arma::vec3({fd.goalpost_own_r[0], fd.goalpost_own_r[1], 0}));
        }
        if (opp && left) {
            possibilities.push_back(arma::vec3({fd.goalpost_opp_l[0], fd.goalpost_opp_l[1], 0}));
        }
        if (opp && right) {
            possibilities.push_back(arma::vec3({fd.goalpost_opp_r[0], fd.goalpost_opp_r[1], 0}));
        }

        return possibilities;
    }
}  // namespace localisation
}  // namespace module
