#include "PlanKick.hpp"

#include <string>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/localisation/FilteredBall.hpp"
#include "message/planning/KickTo.hpp"
#include "message/skill/Kick.hpp"

#include "utility/input/LimbID.hpp"

namespace module::planning {

    using extension::Configuration;
    using message::localisation::FilteredBall;
    using message::planning::KickTo;
    using message::skill::Kick;
    using utility::input::LimbID;

    PlanKick::PlanKick(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("PlanKick.yaml").then([this](const Configuration& config) {
            // Use configuration here from file PlanKick.yaml
            this->log_level             = config["log_level"].as<NUClear::LogLevel>();
            cfg.ball_distance_threshold = config["ball_distance_threshold"].as<float>();
            cfg.ball_angle_threshold    = config["ball_angle_threshold"].as<float>();
            cfg.align                   = config["align"].as<bool>();
            cfg.target_angle_threshold  = config["target_angle_threshold"].as<float>();
            cfg.kick_leg                = config["kick_leg"].as<std::string>();
        });

        on<Provide<KickTo>, Trigger<FilteredBall>>().then([this](const KickTo& kick, const FilteredBall& ball) {
            // CHECK IF CLOSE TO BALL
            // Get the angle and distance to the ball
            float ball_angle    = std::abs(std::atan2(ball.rBTt.y(), ball.rBTt.x()));
            float ball_distance = ball.rBTt.head(2).norm();

            // Need to be near the ball to consider kicking it
            if (ball_distance > cfg.ball_distance_threshold || ball_angle > cfg.ball_angle_threshold) {
                emit(std::make_unique<Idle>());
                return;
            }

            // CHECK IF FACING POINT TO KICK TO
            float align_angle = std::abs(std::atan2(kick.rPTt.y(), kick.rPTt.x()));

            // Don't kick if we should align but we're not aligned to the target
            if (cfg.align && align_angle > cfg.target_angle_threshold) {
                emit(std::make_unique<Idle>());
                return;
            }

            // ALL CHECKS PASSED, KICK!
            // Check if config specifies a leg
            if (cfg.kick_leg == "left") {  // can only kick with left
                emit(std::make_unique<Kick>(LimbID::LEFT_LEG));
                return;
            }
            else if (cfg.kick_leg == "right") {  // can only kick with right
                emit(std::make_unique<Kick>(LimbID::RIGHT_LEG));
                return;
            }

            // Else determine leg based on ball position
            // If the ball is more to the left, then kick with the left leg
            if (ball.rBTt.y() > 0.0) {
                emit(std::make_unique<Kick>(LimbID::LEFT_LEG));
            }
            else {  // ball is more to the right
                emit(std::make_unique<Kick>(LimbID::RIGHT_LEG));
            }
        });
    }


}  // namespace module::planning
