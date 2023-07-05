#include "PlanWalkPath.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/localisation/Ball.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/skill/Kick.hpp"
#include "message/skill/Walk.hpp"

#include "utility/math/comparison.hpp"

namespace module::planning {

    using extension::Configuration;

    using message::localisation::Ball;
    using message::planning::TurnAroundBall;
    using message::planning::TurnOnSpot;
    using message::planning::WalkTo;
    using message::skill::Walk;

    PlanWalkPath::PlanWalkPath(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("PlanWalkPath.yaml").then([this](const Configuration& config) {
            // Use configuration here from file PlanWalkPath.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.max_translational_velocity_magnitude = config["max_translational_velocity_magnitude"].as<double>();
            cfg.min_translational_velocity_magnitude = config["min_translational_velocity_magnitude"].as<double>();
            cfg.acceleration                         = config["acceleration"].as<double>();
            cfg.approach_radius                      = config["approach_radius"].as<double>();
            cfg.y_velocity_enabled                   = config["y_velocity_enabled"].as<bool>();

            cfg.max_angular_velocity = config["max_angular_velocity"].as<double>();
            cfg.min_angular_velocity = config["min_angular_velocity"].as<double>();

            cfg.rotate_velocity   = config["rotate_velocity"].as<double>();
            cfg.rotate_velocity_x = config["rotate_velocity_x"].as<double>();
            cfg.rotate_velocity_y = config["rotate_velocity_y"].as<double>();

            cfg.pivot_ball_velocity   = config["pivot_ball_velocity"].as<double>();
            cfg.pivot_ball_velocity_x = config["pivot_ball_velocity_x"].as<double>();
            cfg.pivot_ball_velocity_y = config["pivot_ball_velocity_y"].as<double>();
        });

        // Path to walk to a particular point
        on<Provide<WalkTo>>().then([this](const WalkTo& walk_to) {
            // If robot getting close to the point, begin to decelerate to minimum velocity
            if (walk_to.rPRr.head(2).norm() < cfg.approach_radius) {
                velocity_magnitude -= cfg.acceleration;
                velocity_magnitude = std::max(velocity_magnitude, cfg.min_translational_velocity_magnitude);
            }
            else {
                // If robot is far away from the point, accelerate to max velocity
                velocity_magnitude += cfg.acceleration;
                velocity_magnitude = std::max(cfg.min_translational_velocity_magnitude,
                                              std::min(velocity_magnitude, cfg.max_translational_velocity_magnitude));
            }

            // Obtain the unit vector to desired target in robot space and scale by cfg.translational_velocity
            Eigen::Vector3d velocity_target = Eigen::Vector3d::Zero();

            if (cfg.y_velocity_enabled) {
                velocity_target = walk_to.rPRr.normalized() * velocity_magnitude;
            }
            else {
                velocity_target = Eigen::Vector3d::UnitX() * velocity_magnitude;
            }

            // Set the angular velocity component of the velocity_target with the angular displacement and saturate with
            // value cfg.max_angular_velocity
            velocity_target.z() =
                utility::math::clamp(cfg.min_angular_velocity, walk_to.heading, cfg.max_angular_velocity);

            emit<Task>(std::make_unique<Walk>(velocity_target));
        });

        on<Provide<TurnOnSpot>>().then([this](const TurnOnSpot& turn_on_spot) {
            // Determine the direction of rotation
            int sign = turn_on_spot.clockwise ? -1 : 1;

            // Turn on the spot
            emit<Task>(std::make_unique<Walk>(
                Eigen::Vector3d(cfg.rotate_velocity_x, cfg.rotate_velocity_y, sign * cfg.rotate_velocity)));

            velocity_magnitude = cfg.min_translational_velocity_magnitude;
        });

        on<Provide<TurnAroundBall>>().then([this](const TurnAroundBall& turn_around_ball) {
            // Determine the direction of rotation
            int sign = turn_around_ball.clockwise ? -1 : 1;
            // Turn around the ball
            emit<Task>(std::make_unique<Walk>(Eigen::Vector3d(cfg.pivot_ball_velocity_x,
                                                              sign * cfg.pivot_ball_velocity_y,
                                                              sign * cfg.pivot_ball_velocity)));

            velocity_magnitude = cfg.min_translational_velocity_magnitude;
        });
    }
}  // namespace module::planning
