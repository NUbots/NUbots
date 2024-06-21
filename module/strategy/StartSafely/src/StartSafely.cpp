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
            cfg.start_delay = config["start_delay"].as<double>();

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

        on<Provide<StartSafelyTask>, Uses<Body>, Needs<Body>, Every<10, Per<std::chrono::seconds>>>().then(
            [this](const Uses<Body>& body) {
                // Wait for the subcontroller to connect
                // HardwareIO isn't part of Director, so it doesn't know if the targets were never sent to the motors
                if (NUClear::clock::now() - startup_time < std::chrono::seconds(cfg.start_delay)) {
                    log<NUClear::TRACE>("Waiting for startup");
                    return;
                }

                // Start safely is done once the robot has moved safely to the starting position
                if (body.done) {
                    emit<Task>(std::make_unique<Done>());
                    log<NUClear::INFO>("Completed start safely.");
                }
                // Not requested Body yet and ready to request
                else if (body.run_state == GroupInfo::RunState::NO_TASK) {
                    log<NUClear::INFO>("Starting safely.");

                    NUClear::clock::time_point time = NUClear::clock::now() + std::chrono::seconds(cfg.move_time);

                    auto body = std::make_unique<Body>();

                    for (int i = 0; i < 20; i++) {
                        log<NUClear::TRACE>("Setting servo",
                                            i,
                                            "to",
                                            cfg.servo_targets[i],
                                            "with gain",
                                            cfg.servo_gain);
                        body->servos[i] = ServoCommand(time, cfg.servo_targets[i], ServoState(cfg.servo_gain, 100));
                    }

                    emit<Task>(body);
                }
                // Not done and already requested - idle
                else {
                    emit<Task>(std::make_unique<Idle>());
                }
            });
    }

}  // namespace module::strategy
