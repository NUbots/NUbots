/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "ScriptOptimizer.h"

#include "extension/Script.h"

#include "message/motion/ServoWaypoint.h"
#include "message/platform/darwin/DarwinSensors.h"
#include "message/research/scriptoptimizer/OptimizeScript.h"
#include "message/research/scriptoptimizer/OptimizeScriptResult.h"

namespace module {
namespace research {

    using extension::ExecuteScript;
    using extension::Script;

    using message::motion::AllServoWaypointsComplete;
    using message::platform::darwin::DarwinSensors;
    using message::research::scriptoptimizer::OptimizeScript;
    using message::research::scriptoptimizer::OptimizeScriptResult;

    ScriptOptimizer::ScriptOptimizer(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), recording(false) {

        on<Startup>().then([this] {
            emit(std::make_unique<OptimizeScriptResult>());
            NUClear::log<NUClear::DEBUG>("Requesting initial script");
        });

        on<Network<OptimizeScript>, With<NUClear::extensions::NetworkingConfiguration>>(
            [this](const Network<OptimizeScript>& task, const NUClear::extensions::NetworkingConfiguration config) {
                // Check if this script is for us
                if (config.deviceName == task.data->target()) {

                    NUClear::log<NUClear::DEBUG>(
                        "Script ", task.data->iteration(), " was delivered to be executed from ", task.sender);

                    Script script;

                    // Make a script from the frames
                    for (const auto& frame : task.data->frames()) {
                        Script::Frame f;

                        f.duration = std::chrono::milliseconds(frame.duration());

                        for (const auto& target : frame.targets()) {
                            Script::Frame::Target t;

                            t.id       = static_cast<message::input::ServoID>(target.id());
                            t.position = target.position();
                            t.gain     = target.gain();

                            f.targets.push_back(std::move(t));
                        }

                        script.frames.push_back(std::move(f));
                    }

                    // Stop recording
                    this->recording = false;

                    this->iteration = task.data->iteration();
                    this->metadata  = task.data->metadata();

                    // Clear the sensors
                    sensors.clear();

                    // Emit our script
                    emit(std::make_unique<ExecuteScript>(script));

                    // Start recording
                    this->recording = true;
                }
            });

        on<Trigger<DarwinSensors>>([this](std::shared_ptr<const DarwinSensors> frame) {
            // While we are recording, store all the frames in a vector
            if (this->recording) {
                sensors.push_back(frame);
            }
        });

        on<Trigger<AllServoWaypointsComplete>>().then([this](const AllServoWaypointsComplete&) {
            // If we were recording
            if (this->recording) {

                // Stop recording
                this->recording = false;

                // Make a response message to the optimizer
                auto result = std::make_unique<OptimizeScriptResult>();

                result->set_iteration(iteration);
                result->set_metadata(metadata);

                // Store all the sensor values for the script
                for (const auto& sensor : sensors) {

                    auto* s = result->add_sensors();

                    auto* acc = s->mutable_accelerometer();
                    acc->set_x(sensor->accelerometer.x);
                    acc->set_y(sensor->accelerometer.y);
                    acc->set_z(sensor->accelerometer.z);

                    auto* gyro = s->mutable_gyroscope();
                    gyro->set_x(sensor->gyroscope.x);
                    gyro->set_y(sensor->gyroscope.y);
                    gyro->set_z(sensor->gyroscope.z);

                    auto* lfsr = s->mutable_left_fsr();
                    lfsr->set_fsr1(sensor->fsr.left.fsr1);
                    lfsr->set_fsr2(sensor->fsr.left.fsr2);
                    lfsr->set_fsr3(sensor->fsr.left.fsr3);
                    lfsr->set_fsr4(sensor->fsr.left.fsr4);
                    lfsr->set_centre_x(sensor->fsr.left.centreX);
                    lfsr->set_centre_y(sensor->fsr.left.centreY);

                    auto* rfsr = s->mutable_right_fsr();
                    rfsr->set_fsr1(sensor->fsr.right.fsr1);
                    rfsr->set_fsr2(sensor->fsr.right.fsr2);
                    rfsr->set_fsr3(sensor->fsr.right.fsr3);
                    rfsr->set_fsr4(sensor->fsr.right.fsr4);
                    rfsr->set_centre_x(sensor->fsr.right.centreX);
                    rfsr->set_centre_y(sensor->fsr.right.centreY);

                    for (int i = 0; i < 20; ++i) {
                        auto* servo = s->add_servo();

                        servo->set_error_flags(sensor->servo[i].errorFlags);
                        servo->set_id(static_cast<message::input::Sensors_ServoID>(i));
                        servo->set_enabled(sensor->servo[i].torqueEnabled);
                        servo->set_p_gain(sensor->servo[i].pGain);
                        servo->set_i_gain(sensor->servo[i].iGain);
                        servo->set_d_gain(sensor->servo[i].dGain);
                        servo->set_goal_position(sensor->servo[i].goalPosition);
                        servo->set_goal_speed(sensor->servo[i].movingSpeed);
                        servo->set_torque_limit(sensor->servo[i].torqueLimit);
                        servo->set_present_position(sensor->servo[i].presentPosition);
                        servo->set_present_speed(sensor->servo[i].presentSpeed);
                        servo->set_load(sensor->servo[i].load);
                        servo->set_voltage(sensor->servo[i].voltage);
                        servo->set_temperature(sensor->servo[i].temperature);
                    }
                }

                // Return our result to the optimizer
                emit<Scope::NETWORK>(std::move(result));
            }
        });
    }
}  // namespace research
}  // namespace module
