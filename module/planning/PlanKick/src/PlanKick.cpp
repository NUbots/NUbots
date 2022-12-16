#include "PlanKick.hpp"

#include "extension/Configuration.hpp"

namespace module::planning {

    using extension::Configuration;

    PlanKick::PlanKick(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("PlanKick.yaml").then([this](const Configuration& config) {
            // Use configuration here from file PlanKick.yaml
            this->log_level        = config["log_level"].as<NUClear::LogLevel>();
            cfg.distance_threshold = config["distance_threshold"].as<float>();
            cfg.angle_threshold    = config["distance_threshold"].as<float>();
        });
    }


    on<Provide<CheckCloseToBall>, Trigger<FilteredBall>>().then([this](const RunInfo& info, const FilteredBall& ball) {
        // Get the angle and distance to the ball
        float absolute_yaw_angle = std::abs(std::atan2(ball.rBTt.y(), ball.rBTt.x()));
        float distance_to_ball   = ball.rBTt.head(2).norm();

        // If the ball is within parameters, then we are Done checking
        if (distance_to_ball < cfg.distance_threshold && absolute_yaw_angle < cfg.angle_threshold) {
            // Save the position of the ball for when kicking, if no further ball detections exist
            rBTt = ball.rBTt;
            emit<Task>(std::make_unique<Done>());
            return;
        }

        // Otherwise, we wait for the ball to be closer
        emit<Task>(std::make_unique<Idle>());
    });

    on<Provide<KickBall>, With<FilteredBall>>().then([this](const RunInfo& info,
                                                            const std::shared_ptr<const FilteredBall>& ball) {
        // If the Kick Task is Done, then KickBall is also Done
        if (info.run_reason == RunInfo::RunReason::SUBTASK_DONE) {
            emit<Task>(std::make_unique<Done>());
            return;
        }

        // If there are no balls, something is wrong! We can't kick the ball if there are no balls.
        if (!ball) {
            emit<Task>(std::make_unique<Done>());
            return;
        }

        // If the ball is more to the left, then kick with the left leg
        if (ball.rBTt.y() > 0.0) {
            emit(std::make_unique<Kick>(LimbID::LEFT_LEG));
        }
        else {  // ball is more to the right
            emit(std::make_unique<Kick>(LimbID::RIGHT_LEG));
        }
    });

}  // namespace module::planning
