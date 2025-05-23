#include "Defend.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/localisation/Field.hpp"
#include "message/localisation/Robot.hpp"
#include "message/purpose/Player.hpp"
#include "message/strategy/FindBall.hpp"
#include "message/strategy/LookAtFeature.hpp"
#include "message/strategy/WalkToBall.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"
#include "message/support/FieldDescription.hpp"

#include "utility/math/euler.hpp"
#include "utility/strategy/positioning.hpp"

namespace module::purpose {

    using extension::Configuration;
    using message::localisation::Robots;
    using DefendMsg = message::purpose::Defend;
    using message::localisation::Field;
    using message::strategy::FindBall;
    using message::strategy::LookAtBall;
    using message::strategy::TackleBall;
    using message::strategy::WalkToFieldPosition;
    using message::support::FieldDescription;
    using utility::math::euler::pos_rpy_to_transform;
    using utility::strategy::optimal_pos;

    Defend::Defend(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Defend.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Defend.yaml
            this->log_level     = config["log_level"].as<NUClear::LogLevel>();
            cfg.clearance       = config["clearance"].as<double>();
            cfg.equidist_weight = config["equidist_weight"].as<double>();
            cfg.neutral_weight  = config["neutral_weight"].as<double>();
            cfg.avoid_weight    = config["avoid_weight"].as<double>();
            cfg.max_iter        = config["max_iter"].as<int>();
            cfg.step_size       = config["step_size"].as<double>();
        });

        on<Provide<DefendMsg>, With<Robots>, With<Field>, With<FieldDescription>>().then(
            [this](const Robots& robots, const Field& field, const FieldDescription& field_desc) {
                // General tasks
                emit<Task>(std::make_unique<FindBall>(), 1);    // Need to know where the ball is
                emit<Task>(std::make_unique<LookAtBall>(), 2);  // Track the ball

                // In this state, we are expected to hang back in the case that we become the attacking robot
                // Stick near an opponent if they are in the first third of the field
                std::vector<Eigen::Vector3d> threatening_opponents =
                    opponents_in_first_third(robots, field, field_desc);

                // Collect all robots in a vector
                std::vector<Eigen::Vector3d> all_robots{};
                for (const auto& robot : robots.robots) {
                    all_robots.push_back(field.Hfw * robot.rRWw);
                }

                // The middle of the front of the penalty area is the defender's neutral spot
                Eigen::Vector3d neutral_point(
                    (field_desc.dimensions.field_length / 2.0) - field_desc.dimensions.penalty_area_length,
                    0.0,
                    0.0);

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

                // Walk to the optimal position
                emit<Task>(std::make_unique<WalkToFieldPosition>(
                    pos_rpy_to_transform(optimal_position, Eigen::Vector3d(0, 0, M_PI)),
                    true));
            });
    }

    std::vector<Eigen::Vector3d> Defend::opponents_in_first_third(const Robots& robots,
                                                                  const Field& field,
                                                                  const FieldDescription& field_desc) {
        // If between our goal line and the first third of the field, we can mark them
        const double first_third = field_desc.dimensions.field_length / (2.0 * 3.0);
        const double goal_line   = field_desc.dimensions.field_length / 2.0;

        // Check if any opponents are in the first third of the field
        std::vector<Eigen::Vector3d> opponents_in_first_third{};
        for (const auto& robot : robots.robots) {
            // Add only opponents and if within the bounds
            Eigen::Vector3d rRFf = field.Hfw * robot.rRWw;
            if (robot.teammate_id == 0 && rRFf.x() > first_third && rRFf.x() < goal_line) {
                opponents_in_first_third.push_back(rRFf);
            }
        }
        return opponents_in_first_third;
    }

}  // namespace module::purpose
