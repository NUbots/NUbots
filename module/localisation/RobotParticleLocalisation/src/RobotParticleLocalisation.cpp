#include "RobotParticleLocalisation.hpp"

#include <Eigen/Geometry>
#include <fmt/format.h>
#include <fmt/ostream.h>

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
                Eigen::Vector3d state(filter.get());
                emit(graph("robot filter state = ", state.x(), state.y(), state.z()));

                // Emit state
                auto field(std::make_unique<Field>());
                Eigen::Affine2d position(Eigen::Affine2d::Identity());
                position.translation() = Eigen::Vector2d(state[RobotModel<double>::kX], state[RobotModel<double>::kY]);
                position.linear()      = Eigen::Rotation2Dd(state[RobotModel<double>::kAngle]).toRotationMatrix();
                field->position        = position.matrix();
                field->covariance      = filter.getCovariance();

                if (config.debug) {
                    log<NUClear::DEBUG>(fmt::format("Robot Location {} : {} : {}",
                                                    state[RobotModel<double>::kX],
                                                    state[RobotModel<double>::kY],
                                                    state[RobotModel<double>::kAngle]));
                }

                emit(std::make_unique<std::vector<Field>>(1, *field));
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

                    // Save the filter state
                    auto saved_filter = filter;

                    // Go through all of the known sided posts
                    for (auto goal_post : goals.goals) {
                        if (goal_post.side != Goal::Side::UNKNOWN_SIDE) {
                            // These are either left or right goal posts

                            // Compare each of these to the possible goal posts on the field (own and opp)

                            // TODO remove the repeated measurements
                            auto m = goal_post.measurements[0];

                            // Check that the measurement is sane
                            if (m.position.allFinite() && m.covariance.allFinite()) {
                                auto filter_new_own = saved_filter;

                                // Make a new candidate for the post
                                auto candidate_own =
                                    filter_new_own.measure(Eigen::Vector3d(m.position.cast<double>()),
                                                           Eigen::Matrix3d(m.covariance.cast<double>()),
                                                           getFieldPosition(goal_post, fd, 1),  // Own post
                                                           goals.Hcw);                          //,
                                                                                                //    m.type,
                                                                                                //    fd);

                                auto filter_new_opp = saved_filter;

                                // Make a new candidate for the post
                                auto candidate_opp =
                                    filter_new_opp.measure(Eigen::Vector3d(m.position.cast<double>()),
                                                           Eigen::Matrix3d(m.covariance.cast<double>()),
                                                           getFieldPosition(goal_post, fd, 0),  // Opp post
                                                           goals.Hcw);                          //,
                                //    m.type,
                                //    fd);
                                if (config.debug) {

                                    Eigen::Vector3d state(filter.get());
                                    Eigen::Transform<double, 3, Eigen::Affine> Hfw;
                                    Hfw.translation() = Eigen::Matrix<double, 3, 1>(state.x(), state.y(), 0);
                                    Hfw.linear() =
                                        Eigen::AngleAxis<double>(state.z(), Eigen::Matrix<double, 3, 1>::UnitZ())
                                            .toRotationMatrix();

                                    const Eigen::Transform<double, 3, Eigen::Affine> Hcf(goals.Hcw
                                                                                         * Hfw.inverse().matrix());

                                    const Eigen::Matrix<double, 3, 1> rGCc_own(Hcf
                                                                               * getFieldPosition(goal_post, fd, 1));
                                    const Eigen::Matrix<double, 3, 1> rGCc_opp(Hcf
                                                                               * getFieldPosition(goal_post, fd, 0));
                                    log(fmt::format("Testing post {}",
                                                    goal_post.side == Goal::Side::LEFT ? "left" : "Right"));
                                    log(fmt::format("Candidate own {}", candidate_own));
                                    log(fmt::format("State {}", state.transpose()));
                                    log(fmt::format("Hcw {}\n", goals.Hcw));
                                    log(fmt::format("Actual own post at {}",
                                                    cartesianToSpherical(rGCc_own).transpose()));
                                    log(fmt::format("Candidate opp {}", candidate_opp));
                                    log(fmt::format("Actual opp post at {}",
                                                    cartesianToSpherical(rGCc_opp).transpose()));
                                    log(fmt::format("Measured post at {}", m.position.transpose()));
                                }

                                filter = candidate_own > candidate_opp ? filter_new_own : filter_new_opp;
                            }
                            else {
                                log("Received non-finite measurements from vision. Discarding ...");
                            }
                        }
                    }

                    for (auto goal_post : goals.goals) {
                        if (goal_post.side == Goal::Side::UNKNOWN_SIDE) {
                            log("This isn't handled yet :D");
                        }

                        // If the goal measurement is a pair of goals. Then get the pairs of true goals, for each pair
                        // of true goals, calculate the likelyhood of it being the goal detected. Update the state with
                        // the minimum error state pair

                        // Else if the goal measurement is not a pair of goals. Then get each true goals, calculate the
                        // likelyhood of it being the goal detected. Update the state with the minimum error state

                        // // Check side and team
                        // const Eigen::Vector3d rGFf = getFieldPosition(goal, fd);

                        // for (auto& m : goal.measurements) {

                        //     if (m.type == VisionGoal::MeasurementType::CENTRE) {
                        //         if (m.position.allFinite() && m.covariance.allFinite()) {
                        //             filter.measure(Eigen::Vector3d(m.position.cast<double>()),
                        //                            Eigen::Matrix3d(m.covariance.cast<double>()),
                        //                            rGFf,
                        //                            goals.Hcw);
                        //         }
                        //         else {
                        //             log("Received non-finite measurements from vision. Discarding ...");
                        //         }
                        //     }
                        // }
                    }
                }
            });

        on<Trigger<ResetRobotHypotheses>, With<Sensors>, Sync<RobotParticleLocalisation>>().then(
            "Reset Robot Hypotheses",
            [this](const ResetRobotHypotheses& locReset, const Sensors& sensors) {
                if (locReset.hypotheses.empty()) {
                    filter.set_state(config.start_state,
                                     std::vector<Eigen::Vector3d>(config.start_state.size(), config.start_variance));
                    return;
                }

                std::vector<Eigen::Vector3d> states;
                std::vector<Eigen::Matrix3d> cov;

                Eigen::Affine3d Htw(sensors.Htw);
                log("Reset Robot Hypotheses");
                for (auto& s : locReset.hypotheses) {

                    // Calculate the reset state
                    Eigen::Affine3d Hft;
                    Hft.translation() = Eigen::Vector3d(s.position.x(), s.position.y(), 0);
                    // Linear part of transform is `s.heading` radians rotation about Z axis
                    Hft.linear() = Eigen::AngleAxisd(s.heading, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                    const Eigen::Affine3d Hfw(Hft * Htw);

                    const Eigen::Affine2d hfw_2d_projection(
                        utility::localisation::projectTo2D(Hfw, Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX()));

                    const Eigen::Vector3d hfw_state_vec(hfw_2d_projection.translation().x(),
                                                        hfw_2d_projection.translation().y(),
                                                        Eigen::Rotation2Dd(hfw_2d_projection.rotation()).angle());

                    states.push_back(hfw_state_vec);

                    // Calculate the reset covariance
                    const Eigen::Rotation2D<double> Hfw_xy(
                        utility::localisation::projectTo2D(Hfw, Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX())
                            .rotation());

                    const Eigen::Rotation2D<double> pos_cov(Hfw_xy * s.position_cov * Hfw_xy.matrix().transpose());

                    Eigen::Matrix<double, 3, 3> state_cov(Eigen::Matrix<double, 3, 3>::Identity());
                    state_cov.topLeftCorner(2, 2) = pos_cov.matrix();
                    state_cov(2, 2)               = s.heading_var;
                    cov.push_back(state_cov);
                }
                filter.set_state(states, cov);
            });

        on<Configuration>("RobotParticleLocalisation.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file RobotParticleLocalisation.yaml
            filter.model.processNoiseDiagonal = cfg["process_noise_diagonal"].as<Expression>();
            filter.model.n_rogues             = cfg["n_rogues"].as<int>();
            filter.model.resetRange           = cfg["reset_range"].as<Expression>();
            filter.model.n_particles          = cfg["n_particles"].as<int>();

            config.debug = cfg["debug"].as<bool>();

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
            //     (-fd.dimensions.field_width / 2.0) + ((fd.dimensions.field_width - fd.dimensions.goal_width) / 4.0),
            //     0.0);

            // // Right side goal line
            // config.start_state.emplace_back(
            //     (-fd.dimensions.field_length / 2.0),
            //     (fd.dimensions.field_width / 2.0) - ((fd.dimensions.field_width - fd.dimensions.goal_width) / 4.0),
            //     0.0);

            // Penalty shootout position
            // config.start_state.emplace_back((-fd.dimensions.field_length / 2.0) +
            // fd.dimensions.penalty_mark_distance,
            //                                 0.0,
            //                                 -M_PI);

            filter.set_state(config.start_state,
                             std::vector<Eigen::Vector3d>(config.start_state.size(), config.start_variance));
        });
    }


    // True goal posiiton
    Eigen::Vector3d RobotParticleLocalisation::getFieldPosition(const VisionGoal& goal,
                                                                const message::support::FieldDescription& fd,
                                                                const bool isOwn) const {
        Eigen::Vector3d position;

        const bool left  = (goal.side == VisionGoal::Side::LEFT);
        const bool right = (goal.side == VisionGoal::Side::RIGHT);

        // TODO This should be removed from the message
        // const bool own   = (goal.team != VisionGoal::Team::OPPONENT);
        // const bool opp   = (goal.team != VisionGoal::Team::OWN);

        if (isOwn && left) {
            position = Eigen::Vector3d(fd.goalpost_own_l.x(), fd.goalpost_own_l.y(), 0);
        }
        if (isOwn && right) {
            position = Eigen::Vector3d(fd.goalpost_own_r.x(), fd.goalpost_own_r.y(), 0);
        }
        if (!isOwn && left) {
            position = Eigen::Vector3d(fd.goalpost_opp_l.x(), fd.goalpost_opp_l.y(), 0);
        }
        if (!isOwn && right) {
            position = Eigen::Vector3d(fd.goalpost_opp_r.x(), fd.goalpost_opp_r.y(), 0);
        }

        return position;
    }
}  // namespace module::localisation


// csv recording
// {
//     // clang-format off
//     Eigen::Vector3d position(m.position.cast<double>());
//     std::cout << position(0) << "," << position(1) << "," << position(2);

//     Eigen::Matrix3d cov(m.covariance.cast<double>());
//     std::cout
//         << cov(0, 0) << "," << cov(0, 1) << "," << cov(0, 2) << ","
//         << cov(1, 0) << "," << cov(1, 1) << "," << cov(1, 2) << ","
//         << cov(2, 0) << "," << cov(2, 1) << "," << cov(2, 2) << ",";

//     Eigen::Matrix4d Hcw(goals.Hcw);

//     std::cout
//         << Hcw(0, 0) << "," << Hcw(0, 1) << "," << Hcw(0, 2) << "," << Hcw(0, 3) << ","
//         << Hcw(1, 0) << "," << Hcw(1, 1) << "," << Hcw(1, 2) << "," << Hcw(1, 3) << ","
//         << Hcw(2, 0) << "," << Hcw(2, 1) << "," << Hcw(2, 2) << "," << Hcw(2, 3) << ","
//         << Hcw(3, 0) << "," << Hcw(3, 1) << "," << Hcw(3, 2) << "," << Hcw(3, 3) << ",";

//     std::cout << goal_post.side << std::endl;
//     // clang-format on
// }
