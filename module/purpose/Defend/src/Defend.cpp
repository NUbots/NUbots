#include "Defend.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/localisation/Robots.hpp"
#include "message/purpose/Defend.hpp"
#include "message/strategy/FindBall.hpp"
#include "message/strategy/LookAtBall.hpp"
#include "message/support/FieldDescription.hpp"

#include "utility/strategy/positioning.hpp"

namespace module::purpose {

    using extension::Configuration;
    using message::localisation::Robots;
    using message::purpose::Defend;
    using message::strategy::FindBall;
    using message::strategy::LookAtBall;
    using message::support::FieldDescription;
    using utility::strategy::optimal_pos;

    Defend::Defend(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Defend.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Defend.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Purpose<Defend>, With<Robots>, With<FieldDescription>>().then(
            [this](const Defend& defend, const Robots& robots, const FieldDescription& field) {
                // General tasks
                emit<Task>(std::make_unique<FindBall>(), 1);    // Need to know where the ball is
                emit<Task>(std::make_unique<LookAtBall>(), 2);  // Track the ball

                // In this state, we are expected to hang back in the case that we become the attacking robot
                // Stick near an opponent if they are in the first third of the field
                std::vector<Eigen::Vector3d> threatening_opponents = opponents_in_first_third(robots, field);

                // Collect all robots in a vector
                std::vector<Eigen::Vector3d> all_robots{};
                for (const auto& robot : robots.robots) {
                    all_robots.push_back(robot.rRFf);
                }

                // The middle of the front of the penalty area is the defender's neutral spot
                Eigen::Vector3d neutral_point(
                    (-field.dimensions.field_length / 2.0) + field.dimensions.penalty_area_length,
                    field.dimensions.field_width / 2.0,
                    0);

                // Get the optimal position to defend from
                Eigen::Vector3d optimal_position = optimal_pos(neutral_point,
                                                               neutral_point,
                                                               threatening_opponents,
                                                               all_robots,
                                                               cfg.clearance,
                                                               cfg.equidist_weight,
                                                               cfg.neutral_weight,
                                                               cfg.avoid_weight,
                                                               cfg.step_size,
                                                               cfg.max_iter);

                // todo: face the ball or just up the field
                // Walk to the optimal position
                emit<Task>(std::make_unique<WalkToFieldPosition>(
                    pos_rpy_to_transform(optimal_position, Eigen::Vector3d(0, 0, 0)),
                    true));
            });
    }

    std::vector<Eigen::Vector3d> Defend::opponents_in_first_third(const Robots& robots, const FieldDescription& field) {
        // If between our goal line and the first third of the field, we can mark them
        const double first_third = -field.dimensions.field_length / (2.0 * 3.0);
        const double goal_line   = -field.dimensions.field_length / 2.0;

        // Check if any opponents are in the first third of the field
        std::vector<Eigen::Vector3d> opponents_in_first_third{};
        for (const auto& robot : robots.robots) {
            // Add only opponents and if within the bounds
            if (robot.teammate_id == 0 && robot.rRFf.x() > first_third && robot.rRFf.x() < goal_line) {
                opponents_in_first_third.push_back(robot.rRFf);
            }
        }
        return opponents_in_first_third;
    }

}  // namespace module::purpose
