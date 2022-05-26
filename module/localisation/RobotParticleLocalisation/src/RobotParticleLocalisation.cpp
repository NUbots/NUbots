#include "RobotParticleLocalisation.hpp"

#include <Eigen/Geometry>
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <initializer_list>
#include <limits>
#include <utility>

#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"
#include "message/localisation/ResetRobotHypotheses.hpp"
#include "message/vision/Goal.hpp"

#include "utility/localisation/transform.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::localisation {

    using extension::Configuration;

    using message::input::Sensors;
    using message::localisation::Field;
    using message::localisation::ResetRobotHypotheses;
    using message::support::FieldDescription;
    using VisionGoal  = message::vision::Goal;
    using VisionGoals = message::vision::Goals;

    using utility::math::coordinates::cartesianToSpherical;
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
                const auto curr_time  = NUClear::clock::now();
                const double seconds  = duration_cast<duration<double>>(curr_time - last_time_update_time).count();
                last_time_update_time = curr_time;

                filter.time(seconds);

                // Get filter state and transform
                Eigen::Vector3d state(filter.getMean());
                emit(graph("robot filter state = ", state.x(), state.y(), state.z()));

                // Emit state
                auto field(std::make_unique<Field>());
                Eigen::Affine2d position(Eigen::Affine2d::Identity());
                position.translation() = Eigen::Vector2d(state[RobotModel<double>::kX], state[RobotModel<double>::kY]);
                position.linear()      = Eigen::Rotation2Dd(state[RobotModel<double>::kAngle]).toRotationMatrix();
                field->position        = position.matrix();
                field->covariance      = filter.getCovariance();

                log<NUClear::DEBUG>(fmt::format("Robot Location x {} : y {} : theta {}",
                                                state[RobotModel<double>::kX],
                                                state[RobotModel<double>::kY],
                                                state[RobotModel<double>::kAngle]));

                emit(field);
            });

        on<Trigger<VisionGoals>, With<FieldDescription>, Sync<RobotParticleLocalisation>>().then(
            "Measurement Update",
            [this](const VisionGoals& goals, const FieldDescription& fd) {
                if (!goals.goals.empty()) {
                    // Perform time update
                    using namespace std::chrono;
                    const auto curr_time  = NUClear::clock::now();
                    const double seconds  = duration_cast<duration<double>>(curr_time - last_time_update_time).count();
                    last_time_update_time = curr_time;

                    filter.time(seconds);

                    // Currently this has no idea about pairs of posts, only that it has been labelled left, right and
                    // unknown

                    // Go through all of the known sided posts
                    for (const auto& goal_post : goals.goals) {

                        // Go through all of the measurement types we have
                        for (const auto& m : goal_post.measurements) {

                            // Check that the measurement is finite
                            if (m.srGCc.allFinite() && m.covariance.allFinite()) {
                                if (goal_post.side != VisionGoal::Side::UNKNOWN_SIDE) {
                                    // These are either left or right goal posts

                                    // Compare each of these to the possible goal posts on the field (own and opp)

                                    // Run a measurement for our own goal post and store how likely the filter thinks
                                    // this is a real measurement
                                    auto own_filter = filter;
                                    const auto own_logits =
                                        own_filter.measure(Eigen::Vector3d(m.srGCc.cast<double>()),
                                                           Eigen::Matrix3d(m.covariance.cast<double>()),
                                                           getFieldPosition(goal_post.side, fd, true),
                                                           goals.Hcw);

                                    // Run a measurement for the opposition goal post and store how likely the filter
                                    // thinks this is a real measurement
                                    auto opp_filter = filter;
                                    const auto opp_logits =
                                        opp_filter.measure(Eigen::Vector3d(m.srGCc.cast<double>()),
                                                           Eigen::Matrix3d(m.covariance.cast<double>()),
                                                           getFieldPosition(goal_post.side, fd, false),
                                                           goals.Hcw);

                                    if (log_level <= NUClear::DEBUG) {
                                        const Eigen::Vector3d state(filter.getMean());
                                        Eigen::Affine3d Hfw;
                                        Hfw.translation() = Eigen::Vector3d(state.x(), state.y(), 0);
                                        Hfw.linear() =
                                            Eigen::AngleAxisd(state.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();

                                        const Eigen::Affine3d Hcf(goals.Hcw * Hfw.inverse().matrix());
                                        const Eigen::Vector3d rGCc_own(Hcf
                                                                       * getFieldPosition(goal_post.side, fd, true));
                                        const Eigen::Vector3d rGCc_opp(Hcf
                                                                       * getFieldPosition(goal_post.side, fd, false));

                                        log<NUClear::DEBUG>(
                                            fmt::format("Testing post {}", std::string(goal_post.side)));
                                        log<NUClear::DEBUG>(fmt::format("Candidate own {}", own_logits));
                                        log<NUClear::DEBUG>(fmt::format("State {}", state.transpose()));
                                        log<NUClear::DEBUG>(fmt::format("Hcw {}\n", goals.Hcw));
                                        log<NUClear::DEBUG>(fmt::format("Actual own post at {}",
                                                                        cartesianToSpherical(rGCc_own).transpose()));
                                        log<NUClear::DEBUG>(fmt::format("Candidate opp {}", opp_logits));
                                        log<NUClear::DEBUG>(fmt::format("Actual opp post at {}",
                                                                        cartesianToSpherical(rGCc_opp).transpose()));
                                        log<NUClear::DEBUG>(fmt::format("Measured post at {}", m.srGCc.transpose()));
                                    }

                                    filter = own_logits > opp_logits ? own_filter : opp_filter;
                                }
                                else {
                                    // Keep track of our best option
                                    double best_logits = std::numeric_limits<double>::lowest();
                                    auto best_filter   = filter;

                                    // Check the measurement against each possible goal post and find the best match
                                    for (const auto& post : std::initializer_list<std::pair<VisionGoal::Side, bool>>{
                                             {VisionGoal::Side::LEFT, true},
                                             {VisionGoal::Side::RIGHT, true},
                                             {VisionGoal::Side::LEFT, false},
                                             {VisionGoal::Side::RIGHT, false}}) {
                                        auto current_filter = filter;

                                        const double current_logits =
                                            current_filter.measure(Eigen::Vector3d(m.srGCc.cast<double>()),
                                                                   Eigen::Matrix3d(m.covariance.cast<double>()),
                                                                   getFieldPosition(post.first, fd, post.second),
                                                                   goals.Hcw);

                                        if (current_logits > best_logits) {
                                            best_filter = current_filter;
                                            best_logits = current_logits;
                                        }
                                    }

                                    filter = best_filter;
                                }
                            }
                            else {
                                log<NUClear::WARN>("Received non-finite measurements from vision. Discarding ...");
                            }
                        }
                    }
                }
            });

        on<Trigger<ResetRobotHypotheses>, With<Sensors>, Sync<RobotParticleLocalisation>>().then(
            "Reset Robot Hypotheses",
            [this](const ResetRobotHypotheses& locReset, const Sensors& sensors) {
                std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> hypotheses;
                if (locReset.hypotheses.empty()) {
                    for (const auto& state : config.start_state) {
                        hypotheses.emplace_back(std::make_pair(state, config.start_variance.asDiagonal()));
                    }
                    filter.set_state(hypotheses);
                    return;
                }

                const Eigen::Affine3d Htw(sensors.Htw);
                for (const auto& s : locReset.hypotheses) {

                    // Calculate the reset state
                    Eigen::Affine3d Hft;
                    Hft.translation() = Eigen::Vector3d(s.rTFf.x(), s.rTFf.y(), 0);
                    // Linear part of transform is `s.heading` radians rotation about Z axis
                    Hft.linear() = Eigen::AngleAxisd(s.heading, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                    const Eigen::Affine3d Hfw(Hft * Htw);

                    const Eigen::Affine2d hfw_2d_projection(
                        utility::localisation::projectTo2D(Hfw, Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX()));

                    const Eigen::Vector3d hfw_state_vec(hfw_2d_projection.translation().x(),
                                                        hfw_2d_projection.translation().y(),
                                                        Eigen::Rotation2Dd(hfw_2d_projection.rotation()).angle());

                    // Calculate the reset covariance
                    const Eigen::Rotation2Dd Hfw_xy(
                        utility::localisation::projectTo2D(Hfw, Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX())
                            .rotation());

                    const Eigen::Rotation2Dd pos_cov(Hfw_xy * s.covariance * Hfw_xy.matrix().transpose());

                    Eigen::Matrix3d state_cov(Eigen::Matrix3d::Identity());
                    state_cov.topLeftCorner(2, 2) = pos_cov.matrix();
                    state_cov(2, 2)               = s.heading_var;
                    hypotheses.emplace_back(std::make_pair(hfw_state_vec, state_cov));
                }
                filter.set_state(hypotheses);
            });

        on<Configuration>("RobotParticleLocalisation.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file RobotParticleLocalisation.yaml
            log_level = cfg["log_level"].as<NUClear::LogLevel>();

            filter.model.processNoiseDiagonal = cfg["process_noise_diagonal"].as<Expression>();
            filter.model.n_rogues             = cfg["n_rogues"].as<int>();
            filter.model.resetRange           = cfg["reset_range"].as<Expression>();
            filter.model.n_particles          = cfg["n_particles"].as<int>();

            config.start_variance = cfg["start_variance"].as<Expression>();
        });

        on<Startup, With<FieldDescription>>().then([this](const FieldDescription& fd) {
            // Left side penalty mark
            config.start_state.emplace_back((-fd.dimensions.field_length / 2.0) + fd.dimensions.penalty_mark_distance,
                                            (-fd.dimensions.field_width / 2.0),
                                            -M_PI_2);

            // Right side penalty mark
            config.start_state.emplace_back((-fd.dimensions.field_length / 2.0) + fd.dimensions.penalty_mark_distance,
                                            (fd.dimensions.field_width / 2.0),
                                            M_PI_2);

            // // Left side goal line
            // config.start_state.emplace_back(
            //     (-fd.dimensions.field_length / 2.0),
            //     (-fd.dimensions.field_width / 2.0) + ((fd.dimensions.field_width - fd.dimensions.goal_width)
            //     / 4.0), 0.0);

            // // Right side goal line
            // config.start_state.emplace_back(
            //     (-fd.dimensions.field_length / 2.0),
            //     (fd.dimensions.field_width / 2.0) - ((fd.dimensions.field_width - fd.dimensions.goal_width)
            //     / 4.0), 0.0);

            // Penalty shootout position
            // config.start_state.emplace_back((-fd.dimensions.field_length / 2.0) +
            // fd.dimensions.penalty_mark_distance,
            //                                 0.0,
            //                                 -M_PI);

            std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> hypotheses;
            for (const auto& state : config.start_state) {
                hypotheses.emplace_back(std::make_pair(state, config.start_variance.asDiagonal()));
            }
            filter.set_state(hypotheses);
        });
    }

    // True goal position
    [[nodiscard]] Eigen::Vector3d RobotParticleLocalisation::getFieldPosition(
        const VisionGoal::Side& side,
        const message::support::FieldDescription& fd,
        const bool& isOwn) {
        const bool left  = (side == VisionGoal::Side::LEFT);
        const bool right = (side == VisionGoal::Side::RIGHT);

        if (isOwn && left) {
            return Eigen::Vector3d(fd.goalpost_own_l.x(), fd.goalpost_own_l.y(), 0);
        }
        if (isOwn && right) {
            return Eigen::Vector3d(fd.goalpost_own_r.x(), fd.goalpost_own_r.y(), 0);
        }
        if (!isOwn && left) {
            return Eigen::Vector3d(fd.goalpost_opp_l.x(), fd.goalpost_opp_l.y(), 0);
        }
        if (!isOwn && right) {
            return Eigen::Vector3d(fd.goalpost_opp_r.x(), fd.goalpost_opp_r.y(), 0);
        }

        return Eigen::Vector3d::Zero();
    }
}  // namespace module::localisation
