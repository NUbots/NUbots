#include "AlignRobotToBall.hpp"

#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/strategy/AlignRobotToBall.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::planning::TurnOnSpot;
    using AlignRobotToBallTask = message::strategy::AlignRobotToBall;
    using utility::support::Expression;

    AlignRobotToBall::AlignRobotToBall(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("AlignRobotToBall.yaml").then([this](const Configuration& config) {
            // Use configuration here from file AlignRobotToBall.yaml
            this->log_level                    = config["log_level"].as<NUClear::LogLevel>();
            cfg.start_rotating_angle_threshold = config["start_rotating_angle_threshold"].as<double>();
            cfg.stop_rotating_angle_threshold  = config["stop_rotating_angle_threshold"].as<double>();
            cfg.ball_distance_threshold        = config["ball_distance_threshold"].as<double>();
        });

        on<Provide<AlignRobotToBallTask>,
           Uses<TurnOnSpot>,
           With<Ball>,
           With<Sensors>,
           Every<30, Per<std::chrono::seconds>>>()
            .then([this](const Uses<TurnOnSpot>& turn, const Ball& ball, const Sensors& sensors) {
                // Transform the ball into robot {r} space
                Eigen::Vector3d rBRr = sensors.Hrw * ball.rBWw;

                // Get the angle to the ball
                double ball_distance = rBRr.head(2).norm();
                double ball_angle    = std::abs(std::atan2(rBRr.y(), rBRr.x()));
                log<NUClear::DEBUG>("Ball angle: {}", ball_angle);

                // If the angle to the ball is greater than maximum threshold, and we are not already rotating, rotate
                if (ball_angle > cfg.start_rotating_angle_threshold && ball_distance > cfg.ball_distance_threshold
                    && !rotate) {
                    rotate = true;
                }

                // If the angle to the ball is less than minimum threshold to stop, and we are currently rotating, stop
                // rotating
                if ((ball_angle < cfg.stop_rotating_angle_threshold || ball_distance < cfg.ball_distance_threshold)
                    && rotate) {
                    rotate = false;
                }

                // If the angle to the ball is greater than maximum threshold, rotate to face the ball
                if (rotate && turn.run_state == GroupInfo::RunState::NO_TASK) {
                    // Check if we need to rotate clockwise or anti-clockwise
                    if (rBRr.y() < 0) {
                        emit<Task>(std::make_unique<TurnOnSpot>(true));
                    }
                    else {
                        emit<Task>(std::make_unique<TurnOnSpot>(false));
                    }
                }
                else if (rotate) {  // already rotating and still rotating
                    emit<Task>(std::make_unique<Idle>());
                }
            });
    }

}  // namespace module::strategy
