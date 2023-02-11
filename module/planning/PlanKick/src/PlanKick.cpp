#include "PlanKick.hpp"

#include <string>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/localisation/FilteredBall.hpp"
#include "message/planning/KickTo.hpp"
#include "message/skill/Kick.hpp"

#include "utility/input/LimbID.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::planning {

    using extension::Configuration;
    using message::localisation::FilteredBall;
    using message::planning::KickTo;
    using message::skill::Kick;
    using utility::input::LimbID;
    using utility::support::Expression;

    PlanKick::PlanKick(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("PlanKick.yaml").then([this](const Configuration& config) {
            // Use configuration here from file PlanKick.yaml
            this->log_level             = config["log_level"].as<NUClear::LogLevel>();
            cfg.ball_timeout_threshold  = config["timeout_threshold"].as<float>();
            cfg.ball_distance_threshold = config["ball_distance_threshold"].as<float>();
            cfg.ball_angle_threshold    = config["ball_angle_threshold"].as<float>();
            cfg.target_angle_threshold  = config["target_angle_threshold"].as<float>();
            cfg.kick_leg                = config["kick_leg"].as<std::string>();
        });

        on<Provide<KickTo>, Trigger<FilteredBall>, Needs<Kick>>().then(
            [this](const KickTo& kick, const RunInfo& info, const FilteredBall& ball) {
                // If the kicking subtask is done, then we are no longer kicking
                if (info.run_reason == RunInfo::RunReason::SUBTASK_DONE) {
                    kicking = false;
                }

                // If we're already kicking, don't do anything
                if (kicking) {
                    emit<Task>(std::make_unique<Idle>());
                    return;
                }

                // CHECK IF BALL IS BALL MEASUREMENT IS RECENT ENOUGH
                // If the ball measurement is old, then don't do anything
                auto time_difference = std::chrono::duration_cast<std::chrono::milliseconds>(
                    NUClear::clock::now() - ball.time_of_measurement);
                if (time_difference.count() >= cfg.ball_timeout_threshold) {
                    emit<Task>(std::make_unique<Idle>());
                    return;
                }

                // CHECK IF CLOSE TO BALL
                // Get the angle and distance to the ball
                float ball_angle    = std::abs(std::atan2(ball.rBTt.y(), ball.rBTt.x()));
                float ball_distance = ball.rBTt.head(2).norm();

                // Need to be near the ball to consider kicking it
                if (ball_distance > cfg.ball_distance_threshold || ball_angle > cfg.ball_angle_threshold) {
                    emit<Task>(std::make_unique<Idle>());
                    return;
                }

                // CHECK IF FACING POINT TO KICK TO
                float align_angle = std::abs(std::atan2(kick.rPTt.y(), kick.rPTt.x()));

                // Don't kick if we should align but we're not aligned to the target
                if (align_angle > cfg.target_angle_threshold) {
                    emit<Task>(std::make_unique<Idle>());
                    return;
                }

                // ALL CHECKS PASSED, KICK!
                kicking = true;

                // If the kick leg is forced left, kick left
                // If the kick leg is auto, kick with left leg if ball is more to the left
                if (cfg.kick_leg == LimbID::LEFT_LEG || (cfg.kick_leg == LimbID::UNKNOWN && ball.rBTt.y() > 0.0)) {
                    emit<Task>(std::make_unique<Kick>(LimbID::LEFT_LEG));
                }
                else {  // kick leg is forced right or ball is more to the right and kick leg is auto
                    emit<Task>(std::make_unique<Kick>(LimbID::RIGHT_LEG));
                }
            });
    }


}  // namespace module::planning
