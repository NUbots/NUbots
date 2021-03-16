#include "RobotParticleLocalisation.hpp"

#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"
#include "message/localisation/ResetRobotHypotheses.hpp"
#include "message/vision/Goal.hpp"

#include "utility/localisation/transform.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/eigen_armadillo.hpp"
#include "utility/support/yaml_armadillo.hpp"

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
            "Time Update",
            [this]() {
                /* Perform time update */
                using namespace std::chrono;
                auto curr_time        = NUClear::clock::now();
                double seconds        = duration_cast<duration<double>>(curr_time - last_time_update_time).count();
                last_time_update_time = curr_time;

                filter.time(seconds);

                // Get filter state and transform
                Eigen::Vector3d state(filter.get());
                emit(graph("robot filter state = ", state.x(), state.y(), state.z()));

                // Emit state
                auto field(std::make_unique<Field>());
                Eigen::Affine2d position(Eigen::Affine2d::Identity());
                position.translation() = Eigen::Vector2d(state[RobotModel<double>::kX], state[RobotModel<double>::kY]);
                position.linear()      = Eigen::Rotation2Dd(state[RobotModel<double>::kAngle]).toRotationMatrix();
                field->position        = position.matrix();
                field->covariance      = filter.getCovariance();

                emit(std::make_unique<std::vector<Field>>(1, *field));
                emit(field);
            });

        on<Trigger<VisionGoals>, With<FieldDescription>, Sync<RobotParticleLocalisation>>().then(
            "Measurement Update",
            [this](const VisionGoals& goals, const FieldDescription& fd) {
                if (!goals.goals.empty()) {
                    /* Perform time update */
                    using namespace std::chrono;
                    auto curr_time        = NUClear::clock::now();
                    double seconds        = duration_cast<duration<double>>(curr_time - last_time_update_time).count();
                    last_time_update_time = curr_time;

                    filter.time(seconds);

                    for (auto goal : goals.goals) {

                        // Check side and team
                        Eigen::Vector3d poss = getFieldPosition(goal, fd);

                        for (auto& m : goal.measurements) {
                            if (m.type == VisionGoal::MeasurementType::CENTRE) {
                                if (m.position.allFinite() && m.covariance.allFinite()) {
                                    filter.measure(Eigen::Vector3d(m.position.cast<double>()),
                                                   Eigen::Matrix3d(m.covariance.cast<double>()),
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
            "Reset Robot Hypotheses",
            [this](const ResetRobotHypotheses& locReset, const Sensors& sensors) {
                std::vector<Eigen::Vector3d> states;
                std::vector<Eigen::Matrix<double, 3, 3>> cov;

                const Eigen::Matrix<double, 4, 4>& Htw(sensors.Htw);

                for (auto& s : locReset.hypotheses) {
                    const Eigen::Vector3d rTFf(s.position.x(), s.position.y(), 0);
                    Eigen::Affine3d Hft;
                    Hft.translation() = rTFf;
                    // Linear part of transform is `s.heading` radians rotation about Z axis
                    Hft.linear() = Eigen::AngleAxisd(s.heading, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                    const Eigen::Affine3d Hfw(Hft * Htw);

                    const Eigen::Affine2d hfw_2d_projection(
                        utility::localisation::projectTo2D(Hfw, Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(1, 0, 0)));

                    const Eigen::Vector3d hfw_state_vec(hfw_2d_projection.translation().x(),
                                                        hfw_2d_projection.translation().y(),
                                                        hfw_2d_projection.rotation().angle());

                    states.push_back(hfw_state_vec);

                    const Eigen::Rotation2D<double> Hfw_xy(
                        utility::localisation::projectTo2D(Hfw, Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(1, 0, 0))
                            .rotation());

                    const Eigen::Rotation2D<double> pos_cov(Hfw_xy * s.position_cov * Hfw_xy.matrix().transpose());

                    Eigen::Matrix<double, 3, 3> state_cov(Eigen::Matrix<double, 3, 3>::Identity());
                    state_cov.topLeftCorner(2, 2) = pos_cov.matrix();
                    state_cov(2, 2)               = s.heading_var;
                    cov.push_back(state_cov);
                    filter.set_state(hfw_state_vec, state_cov, n_particles);
                }
            });

        on<Configuration>("RobotParticleLocalisation.yaml").then([this](const Configuration& config) {
            // Use configuration here from file RobotParticleLocalisation.yaml
            filter.model.processNoiseDiagonal = config["process_noise_diagonal"].as<Expression>();
            filter.model.n_rogues             = config["n_rogues"].as<int>();
            filter.model.resetRange           = config["reset_range"].as<Expression>();
            n_particles                       = config["n_particles"].as<int>();
            draw_particles                    = config["draw_particles"].as<int>();

            Eigen::Vector3d start_state = config["start_state"].as<Expression>();
            // TODO: This variable is not used. Probably remove it
            /* Eigen::Vector3d start_variance = config["start_variance"].as<Expression>(); */

            auto reset = std::make_unique<ResetRobotHypotheses>();
            ResetRobotHypotheses::Self leftSide;
            // Start on goal line
            leftSide.position     = Eigen::Vector2d(start_state.x(), start_state.y());
            leftSide.position_cov = Eigen::Vector2d::Constant(0.5).asDiagonal();
            leftSide.heading      = start_state.z();
            leftSide.heading_var  = 0.005;

            reset->hypotheses.push_back(leftSide);
            ResetRobotHypotheses::Self rightSide;
            // Start on goal line
            rightSide.position     = Eigen::Vector2d(start_state.x(), -start_state.y());
            rightSide.position_cov = Eigen::Vector2d::Constant(0.5).asDiagonal();
            rightSide.heading      = -start_state.z();
            rightSide.heading_var  = 0.005;

            reset->hypotheses.push_back(rightSide);
            emit<Scope::DELAY>(reset, std::chrono::seconds(1));
        });
    }

    Eigen::Vector3d RobotParticleLocalisation::getFieldPosition(const VisionGoal& goal,
                                                                const message::support::FieldDescription& fd) const {
        Eigen::Vector3d position;

        const bool left  = (goal.side != VisionGoal::Side::RIGHT);
        const bool right = (goal.side != VisionGoal::Side::LEFT);
        const bool own   = (goal.team != VisionGoal::Team::OPPONENT);
        const bool opp   = (goal.team != VisionGoal::Team::OWN);

        if (own && left) {
            position = Eigen::Vector3d((fd.goalpost_own_l.x(), fd.goalpost_own_l.y(), 0));
        }
        if (own && right) {
            position = Eigen::Vector3d((fd.goalpost_own_r.x(), fd.goalpost_own_r.y(), 0));
        }
        if (opp && left) {
            position = Eigen::Vector3d((fd.goalpost_opp_l.x(), fd.goalpost_opp_l.y(), 0));
        }
        if (opp && right) {
            position = Eigen::Vector3d((fd.goalpost_opp_r.x(), fd.goalpost_opp_r.y(), 0));
        }

        return position;
    }

}  // namespace localisation
}  // namespace module
