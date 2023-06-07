#include "PlanWalkPath.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/localisation/FilteredBall.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/skill/Walk.hpp"

#include "utility/math/comparison.hpp"

namespace module::planning {

    using extension::Configuration;

    using message::localisation::FilteredBall;
    using message::planning::TurnAroundBall;
    using message::planning::TurnOnSpot;
    using message::planning::WalkTo;
    using message::skill::Walk;

    PlanWalkPath::PlanWalkPath(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("PlanWalkPath.yaml").then([this](const Configuration& config) {
            // Use configuration here from file PlanWalkPath.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.max_forward_speed = config["max_forward_speed"].as<double>();
            cfg.min_forward_speed = config["min_forward_speed"].as<double>();
            cfg.acceleration      = config["acceleration"].as<double>();
            cfg.approach_radius   = config["approach_radius"].as<double>();

            cfg.max_turn_speed = config["max_turn_speed"].as<double>();
            cfg.min_turn_speed = config["min_turn_speed"].as<double>();

            cfg.rotate_speed   = config["rotate_speed"].as<double>();
            cfg.rotate_speed_x = config["rotate_speed_x"].as<double>();
            cfg.rotate_speed_y = config["rotate_speed_y"].as<double>();

            cfg.pivot_ball_speed   = config["pivot_ball_speed"].as<double>();
            cfg.pivot_ball_speed_x = config["pivot_ball_speed_x"].as<double>();
            cfg.pivot_ball_speed_y = config["pivot_ball_speed_y"].as<double>();
        });

        // Path to walk to a particular point
        on<Provide<WalkTo>>().then([this](const WalkTo& walk_to) {
            Eigen::Vector3d rPRr = walk_to.rPRr;

            // If robot getting close to the point, begin to decelerate to minimum speed
            if (rPRr.head(2).norm() < cfg.approach_radius) {
                speed -= cfg.acceleration;
                speed = std::max(speed, cfg.min_forward_speed);
            }
            else {
                // If robot is far away from the point, accelerate to max speed
                speed += cfg.acceleration;
                speed = std::max(cfg.min_forward_speed, std::min(speed, cfg.max_forward_speed));
            }

            // Obtain the unit vector to desired target in robot space and scale by cfg.forward_speed
            Eigen::Vector3d walk_command = rPRr.normalized() * speed;

            // Set the angular velocity component of the walk_command with the angular displacement and saturate with
            // value cfg.max_turn_speed
            walk_command.z() = utility::math::clamp(cfg.min_turn_speed, walk_to.heading, cfg.max_turn_speed);

            emit<Task>(std::make_unique<Walk>(walk_command));
        });

        on<Provide<TurnOnSpot>>().then([this](const TurnOnSpot& turn_on_spot) {
            // Determine the direction of rotation
            int sign = turn_on_spot.clockwise ? -1 : 1;

            // Turn on the spot
            emit<Task>(std::make_unique<Walk>(
                Eigen::Vector3d(cfg.rotate_speed_x, cfg.rotate_speed_y, sign * cfg.rotate_speed)));
        });

        on<Provide<TurnAroundBall>>().then([this](const TurnAroundBall& turn_around_ball) {
            // Determine the direction of rotation
            int sign = turn_around_ball.clockwise ? -1 : 1;
            // Turn around the ball
            emit<Task>(std::make_unique<Walk>(
                Eigen::Vector3d(cfg.pivot_ball_speed_x, cfg.pivot_ball_speed_y, sign * cfg.pivot_ball_speed)));
        });
    }
}  // namespace module::planning
