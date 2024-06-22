#include "StartSafely.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/actuation/ServoCommand.hpp"
#include "message/input/Sensors.hpp"
#include "message/strategy/StartSafely.hpp"


namespace module::strategy {

    using StartSafelyTask = message::strategy::StartSafely;
    using message::actuation::Body;
    using message::actuation::ServoCommand;
    using message::actuation::ServoState;
    using message::input::Sensors;

    using extension::Configuration;

    StartSafely::StartSafely(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("StartSafely.yaml").then([this](const Configuration& config) {
            // Use configuration here from file StartSafely.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.move_time   = config["move_time"].as<double>();
            cfg.servo_gain  = config["servo_gain"].as<double>();
            cfg.servo_error = config["servo_error"].as<double>();

            cfg.servo_targets[0]  = config["right_shoulder_pitch"].as<double>();
            cfg.servo_targets[1]  = config["left_shoulder_pitch"].as<double>();
            cfg.servo_targets[2]  = config["right_shoulder_roll"].as<double>();
            cfg.servo_targets[3]  = config["left_shoulder_roll"].as<double>();
            cfg.servo_targets[4]  = config["right_elbow"].as<double>();
            cfg.servo_targets[5]  = config["left_elbow"].as<double>();
            cfg.servo_targets[6]  = config["right_hip_yaw"].as<double>();
            cfg.servo_targets[7]  = config["left_hip_yaw"].as<double>();
            cfg.servo_targets[8]  = config["right_hip_roll"].as<double>();
            cfg.servo_targets[9]  = config["left_hip_roll"].as<double>();
            cfg.servo_targets[10] = config["right_hip_pitch"].as<double>();
            cfg.servo_targets[11] = config["left_hip_pitch"].as<double>();
            cfg.servo_targets[12] = config["right_knee"].as<double>();
            cfg.servo_targets[13] = config["left_knee"].as<double>();
            cfg.servo_targets[14] = config["right_ankle_pitch"].as<double>();
            cfg.servo_targets[15] = config["left_ankle_pitch"].as<double>();
            cfg.servo_targets[16] = config["right_ankle_roll"].as<double>();
            cfg.servo_targets[17] = config["left_ankle_roll"].as<double>();
            cfg.servo_targets[18] = config["head_yaw"].as<double>();
            cfg.servo_targets[19] = config["head_pitch"].as<double>();
        });

        on<Startup>().then([this] {
            // Set a time to complete now, so it won't keep moving further into the future each time Body is requested
            destination_time = NUClear::clock::now() + std::chrono::seconds(cfg.move_time);
            log<NUClear::INFO>("StartSafely is starting");
        });

        on<Provide<StartSafelyTask>, Needs<Body>, Trigger<Sensors>>().then([this](const Sensors& sensors) {
            // Check if servos have reached their positions
            bool all_servos_at_target = true;
            for (auto& servo : sensors.servo) {
                double error = std::abs(servo.present_position - cfg.servo_targets[servo.id]);
                if (error > cfg.servo_error) {
                    log<NUClear::TRACE>("Servo", servo.id, "has not reached target position with error", error);
                    all_servos_at_target = false;
                    break;
                }
            }

            // If they are at their positions, start safely can end
            if (all_servos_at_target) {
                log<NUClear::INFO>("Start safely completed, servos at target positions.");
                emit<Task>(std::make_unique<Done>());
                return;
            }

            // If the servos are not at their positions, request to move
            // Keep requesting as we don't know when the subcontroller will be active
            auto body = std::make_unique<Body>();

            for (int i = 0; i < 20; i++) {
                log<NUClear::TRACE>("Setting servo", i, "to", cfg.servo_targets[i], "with gain", cfg.servo_gain);
                body->servos[i] = ServoCommand(destination_time, cfg.servo_targets[i], ServoState(cfg.servo_gain, 100));
            }

            emit<Task>(body);
        });
    }

}  // namespace module::strategy
