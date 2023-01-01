#include "PlanWalkPath.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/planning/WalkPath.hpp"
#include "message/skill/Walk.hpp"

#include "utility/math/comparison.hpp"

namespace module::planning {

    using extension::Configuration;

    using message::planning::Pivot;
    using message::planning::WalkTo;
    using message::skill::Walk;

    PlanWalkPath::PlanWalkPath(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("PlanWalkPath.yaml").then([this](const Configuration& config) {
            // Use configuration here from file PlanWalkPath.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.speed              = config["speed"].as<float>();
            cfg.max_turn_speed     = config["max_turn_speed"].as<float>();
            cfg.pivot_speed        = config["pivot_speed"].as<float>();
            cfg.pivot_ball_speed_y = config["pivot_ball_speed_y"].as<float>();
        });

        // Path to walk to a particular point
        on<Provide<WalkTo>, Needs<Walk>>().then([this](const WalkTo& walk_to) {
            // Get the point and zero out the z position since we are walking along the ground and want to scale the
            // velocity in the xy plane
            Eigen::Vector3f walk_command = walk_to.rPTt;
            walk_command.z()             = 0.0;
            // Obtain the unit vector to desired target in torso space and scale by cfg.speed
            walk_command = walk_command.normalized() * cfg.speed;

            // Set the angular velocity component of the walk_command with the angular displacement and saturate with
            // value cfg.max_turn_speed
            walk_command.z() = utility::math::clamp(-cfg.max_turn_speed,
                                                    std::atan2(walk_command.y(), walk_command.x()),
                                                    cfg.max_turn_speed);

            emit<Task>(std::make_unique<Walk>(walk_command));
        });

        // Pivoting around a fixed point - could be self (rotating on the spot) or rotating around a ball
        // Very basic functionality currently - could be improved to mathematically determine walk command
        // using a maximum velocity and any pivot point.
        on<Provide<Pivot>, Needs<Walk>>().then([this](const Pivot& pivot) {
            // Determine the direction of rotation
            int sign = pivot.clockwise ? -1 : 1;

            // Pivoting on the spot
            if (pivot.rPTt == Eigen::Vector3f::Zero()) {
                emit<Task>(std::make_unique<Walk>(Eigen::Vector3f(0.0, 0.0, sign * cfg.pivot_speed)));
            }
            else {  // must be pivoting around the ball - assume it is directly in front
                emit<Task>(std::make_unique<Walk>(
                    Eigen::Vector3f(0.0, -sign * cfg.pivot_ball_speed_y, sign * cfg.pivot_speed)));
            }
        });

        // Note: walk to ready is not here, in favour of just emitting a Walk task directly from the strategy for ready
        // state - then the config values can be in that module, which is where ppl are more likely to look for them
    }

}  // namespace module::planning
