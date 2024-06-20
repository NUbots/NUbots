#include "StartSafely.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/ServoTarget.hpp"
#include "message/input/Sensors.hpp"
#include "message/strategy/StartSafely.hpp"


namespace module::strategy {

    using StartSafelyTask = message::strategy::StartSafely;
    using message::actuation::ServoTarget;
    using message::input::Sensors;

    using extension::Configuration;

    StartSafely::StartSafely(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("StartSafely.yaml").then([this](const Configuration& config) {
            // Use configuration here from file StartSafely.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.error       = config["error"].as<double>();
        });

        provider = on<Provide<StartSafelyTask>>()
                       .then([this] {
                           // Emit a zero velocity walk task to make the robot stand
                           log<NUClear::INFO>("Emitting Walk task to make the robot stand safely.");
                           emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero(), 1));
                       })
                       .enabled();

        position_check =
            on<Trigger<ServoTarget>, With<Sensors>>()
                .then([this](const ServoTarget& target, const Sensors& sensors) {
                    // Check if we have seen this servo target before, and if not save the position
                    if (servo_targets.find(target.id) == servo_targets.end()) {
                        servo_targets[target.id] = target.position;
                    }

                    // If we have seen all the servo targets, then we can check if the servos are at their
                    // position
                    if (servo_targets.size() == 20) {
                        // Check if all the servos are at their target position
                        bool all_at_target = true;
                        for (i = 0; i < 20; i++) {
                            double error = std::abs(sensors.servo[i].present_position - servo_targets[i]);
                            if (error > 0.01) {
                                all_at_target = false;
                                break;  // don't keep looking if at least one is not in position
                            }
                        }

                        // If all the servos are at their target position, then start safely can stop
                        if (all_at_target) {
                            log<NUClear::INFO>("All servos are at their target position, stopping StartSafely.");

                            provider.disable();
                            provider.unbind();
                            position_check.disable();
                            position_check.unbind();
                        }
                    }
                })
                .enabled();
    }

}  // namespace module::strategy
