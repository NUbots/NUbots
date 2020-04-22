#include "RobotParticleLocalisation.h"

#include "extension/Configuration.h"
#include "message/input/Sensors.h"
#include "message/localisation/Field.h"
#include "message/localisation/ResetRobotHypotheses.h"
#include "message/vision/Goal.h"
#include "utility/localisation/transform.h"
#include "utility/nusight/NUhelpers.h"
#include "utility/support/yaml_armadillo.h"

namespace module {
namespace localisation {

    using extension::Configuration;

    using message::input::Sensors;
    using message::localisation::Field;
    using message::localisation::ResetRobotHypotheses;
    using message::support::FieldDescription;
    using VisionGoal  = message::vision::Goal;
    using VisionGoals = message::vision::Goals;

    using utility::nusight::graph;
    using utility::support::Expression;

    RobotParticleLocalisation::RobotParticleLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        last_measurement_update_time = NUClear::clock::now();
        last_time_update_time        = NUClear::clock::now();

        on<Every<TIME_UPDATE_FREQUENCY, Per<std::chrono::seconds>>, Sync<RobotParticleLocalisation>>().then(
            "Time Update", [this]() {
                /* Perform time update */
                using namespace std::chrono;
                auto curr_time        = NUClear::clock::now();
                double seconds        = duration_cast<duration<double>>(curr_time - last_time_update_time).count();
                last_time_update_time = curr_time;

                filter.time(seconds);

                // Get filter state and transform
                Eigen::Vector3d state = filter.get();
                emit(graph("robot filter state = ", state[0], state[1], state[2]));

                // Emit state
                auto field               = std::make_unique<Field>();
                Eigen::Affine2d position = Eigen::Affine2d::Identity();
                position.translation() = Eigen::Vector2d(state[RobotModel<double>::kX], state[RobotModel<double>::kY]);
                position.linear()      = Eigen::Rotation2Dd(state[RobotModel<double>::kAngle]).toRotationMatrix();
                field->position        = position.matrix();
                field->covariance      = filter.getCovariance();

                emit(std::make_unique<std::vector<Field>>(1, *field));
                emit(field);
            });

        on<Trigger<VisionGoals>, With<FieldDescription>, Sync<RobotParticleLocalisation>>().then(
            "Measurement Update", [this](const VisionGoals& goals, const FieldDescription& fd) {
                if (!goals.goals.empty()) {
                    /* Perform time update */
                    using namespace std::chrono;
                    auto curr_time        = NUClear::clock::now();
                    double seconds        = duration_cast<duration<double>>(curr_time - last_time_update_time).count();
                    last_time_update_time = curr_time;

                    filter.time(seconds);

                    for (auto goal : goals.goals) {

                        // Check side and team
                        std::vector<Eigen::VectorXd> poss = getPossibleFieldPositions(goal, fd);

                        /* These parameters must be cast because Eigen doesn't do implicit conversion of float to
                           double. They must also be wrapped, because Eigen converts to an intermediate type after the
                           cast and that intermediate type screws up the function call */
                        for (auto& m : goal.measurements) {
                            if (m.type == VisionGoal::MeasurementType::CENTRE) {
                                if (m.position.allFinite() && m.covariance.allFinite()) {
                                    filter.measure(Eigen::VectorXd{m.position.cast<double>()},
                                                   Eigen::MatrixXd{m.covariance.cast<double>()},
                                                   poss,
                                                   goals.Hcw,
                                                   m.type,
                                                   fd);
                                }
                                else {
                                    log("Received non-finite measurements from vision. Discarding ...");
                                }
                            }
                        }
                    }
                }
            });

        on<Trigger<ResetRobotHypotheses>, With<Sensors>, Sync<RobotParticleLocalisation>>().then(
            "Reset Robot Hypotheses", [this](const ResetRobotHypotheses& locReset, const Sensors& sensors) {
                Eigen::Affine3d Hfw;
                const Eigen::Matrix<double, 4, 4>& Htw = sensors.Htw;
                std::vector<Eigen::Vector3d> states;
                std::vector<Eigen::Matrix<double, 3, 3>> cov;

                for (auto& s : locReset.hypotheses) {
                    Eigen::Affine3d Hft;
                    Eigen::Vector3d rTFf              = {s.position[0], s.position[1], 0};
                    Hft.translation()                 = rTFf;
                    Hfw                               = Hft * Htw;
                    Eigen::Affine2d hfw_2d_projection = utility::localisation::projectTo2D(Hfw);
                    Eigen::Vector3d hfw_state_vec     = {
                        hfw_2d_projection.translation()(0), hfw_2d_projection.translation()(1), 0};
                    Eigen::Rotation2D<double> hfw_2d_rotation;
                    hfw_2d_rotation.matrix() = hfw_2d_projection.linear();
                    hfw_state_vec(2)         = hfw_2d_rotation.angle();

                    states.push_back(hfw_state_vec);

                    Eigen::Rotation2D<double> Hfw_xy;
                    Hfw_xy.matrix() = utility::localisation::projectTo2D(Hfw).linear();
                    Eigen::Rotation2D<double> pos_cov;
                    pos_cov.matrix()                      = Hfw_xy * s.position_cov * Hfw_xy.matrix().transpose();
                    Eigen::Matrix<double, 3, 3> state_cov = Eigen::Matrix<double, 3, 3>::Identity();
                    state_cov.block(0, 0, 1, 1)           = pos_cov.matrix();
                    state_cov(2, 2)                       = s.heading_var;
                    cov.push_back(state_cov);
                }
                filter.set_state_ambiguous(states, cov, n_particles);
            });

        on<Configuration>("RobotParticleLocalisation.yaml").then([this](const Configuration& config) {
            // Use configuration here from file RobotParticleLocalisation.yaml
            filter.model.processNoiseDiagonal = config["process_noise_diagonal"].as<Expression>();
            filter.model.n_rogues             = config["n_rogues"].as<int>();
            filter.model.resetRange           = config["reset_range"].as<Expression>();
            n_particles                       = config["n_particles"].as<int>();
            draw_particles                    = config["draw_particles"].as<int>();

            Eigen::Vector3d start_state    = config["start_state"].as<Expression>();
            Eigen::Vector3d start_variance = config["start_variance"].as<Expression>();

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

    std::vector<Eigen::VectorXd> RobotParticleLocalisation::getPossibleFieldPositions(
        const VisionGoal& goal,
        const message::support::FieldDescription& fd) const {
        std::vector<Eigen::VectorXd> possibilities;

        bool left  = (goal.side != VisionGoal::Side::RIGHT);
        bool right = (goal.side != VisionGoal::Side::LEFT);
        bool own   = (goal.team != VisionGoal::Team::OPPONENT);
        bool opp   = (goal.team != VisionGoal::Team::OWN);

        if (own && left) {
            possibilities.push_back(Eigen::Vector3d({fd.goalpost_own_l[0], fd.goalpost_own_l[1], 0}));
        }
        if (own && right) {
            possibilities.push_back(Eigen::Vector3d({fd.goalpost_own_r[0], fd.goalpost_own_r[1], 0}));
        }
        if (opp && left) {
            possibilities.push_back(Eigen::Vector3d({fd.goalpost_opp_l[0], fd.goalpost_opp_l[1], 0}));
        }
        if (opp && right) {
            possibilities.push_back(Eigen::Vector3d({fd.goalpost_opp_r[0], fd.goalpost_opp_r[1], 0}));
        }

        return possibilities;
    }
}  // namespace localisation
}  // namespace module
