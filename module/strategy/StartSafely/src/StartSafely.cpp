/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "StartSafely.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/ServoCommand.hpp"
#include "message/input/Sensors.hpp"
#include "message/strategy/StartSafely.hpp"

#include "utility/skill/Script.hpp"


namespace module::strategy {

    using StartSafelyTask = message::strategy::StartSafely;
    using message::actuation::BodySequence;
    using message::actuation::ServoCommand;
    using message::actuation::ServoState;
    using message::input::Sensors;

    using utility::skill::load_script;

    using extension::Configuration;

    StartSafely::StartSafely(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("StartSafely.yaml").then([this](const Configuration& config) {
            // Use configuration here from file StartSafely.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.move_time   = config["move_time"].as<double>();
            cfg.servo_error = config["servo_error"].as<double>();
            cfg.servo_gain  = config["servo_gain"].as<double>();
            cfg.script_name = config["script_name"].as<std::string>();
            cfg.max_timeout = config["max_timeout"].as<double>();
        });

        on<Startup>().then([this] {
            // Set the startup time for tracking max time
            startup_time = NUClear::clock::now();

            // Load the script
            script = load_script<BodySequence>(cfg.script_name);
            for (int i = 0; i < 20; i++) {
                servo_targets[i]                       = script->frames[0].servos[i].position;
                script->frames[0].servos[i].state.gain = cfg.servo_gain;
                script->frames[0].servos[i].time       = NUClear::clock::now() + std::chrono::seconds(cfg.move_time);
            }
        });

        on<Provide<StartSafelyTask>, Needs<BodySequence>, Trigger<Sensors>>().then([this](const Sensors& sensors) {
            // Hard timeout for start safely so it should never get the system stuck
            if (std::chrono::duration_cast<std::chrono::seconds>(NUClear::clock::now() - startup_time).count()
                > cfg.max_timeout) {
                log<NUClear::INFO>("Start safely ended due to timeout.");
                emit<Task>(std::make_unique<Done>());
                return;
            }

            // Check if servos have reached their positions
            bool all_servos_at_target = true;
            for (auto& servo : sensors.servo) {
                double error = std::abs(servo.present_position - servo_targets[servo.id]);
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

            // This shouldn't happen as it is set on startup
            if (script == nullptr) {
                log<NUClear::ERROR>("Script not loaded.");
                return;
            }

            emit<Task>(script);
        });
    }

}  // namespace module::strategy
