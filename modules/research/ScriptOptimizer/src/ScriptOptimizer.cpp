/*
 * This file is part of ScriptOptimizer.
 *
 * ScriptOptimizer is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ScriptOptimizer is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ScriptOptimizer.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "ScriptOptimizer.h"
#include "messages/motion/Script.h"
#include "messages/motion/ServoWaypoint.h"
#include "messages/research/scriptoptimizer/OptimizeScript.pb.h"
#include "messages/research/scriptoptimizer/OptimizeScriptResult.pb.h"
#include "messages/platform/darwin/DarwinSensors.h"

namespace modules {
    namespace research {
        
        using messages::platform::darwin::DarwinSensors;
        using messages::research::scriptoptimizer::OptimizeScript;
        using messages::research::scriptoptimizer::OptimizeScriptResult;
        using messages::motion::ExecuteScript;
        using messages::motion::AllServoWaypointsComplete;
        using messages::motion::Script;
        
        ScriptOptimizer::ScriptOptimizer(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), recording(false) {

            on<Trigger<Network<OptimizeScript>>, With<NUClear::extensions::NetworkingConfiguration>>([this]
                    (const Network<OptimizeScript>& task, const NUClear::extensions::NetworkingConfiguration config) {

                // Check if this script is for us
                if (config.deviceName == task.data->target()) {
                    
                    log<NUClear::DEBUG>("Script ", task.data->iteration(), " was delivered to be executed");

                    Script script;
                    
                    // Make a script from the frames
                    for (const auto& frame : task.data->frames()) {
                        Script::Frame f;
                        for (const auto& target : frame.targets()) {
                            Script::Frame::Target t;
                            
                            t.id = static_cast<messages::input::ServoID>(target.id());
                            t.position = target.position();
                            t.gain = target.gain();
                            
                            f.targets.push_back(std::move(t));
                        }
                        
                        script.frames.push_back(std::move(f));
                    }
                    
                    // Stop recording
                    this->recording = false;
                    
                    this->iteration = task.data->iteration();
                    this->metadata = task.data->metadata();
                    
                    // Clear the sensors
                    sensors.clear();
                    
                    // Emit our script
                    emit(std::make_unique<ExecuteScript>(script));
                    
                    // Start recording
                    this->recording = true;
                }
            });
            
            on<Trigger<Raw<DarwinSensors>>>([this](const std::shared_ptr<const DarwinSensors>& frame) {
                
                // While we are recording, store all the frames in a vector
                if(this->recording) {
                    sensors.push_back(frame);
                }
            });
            
            on<Trigger<AllServoWaypointsComplete>>([this](const AllServoWaypointsComplete&) {
                
                // If we were recording
                if(this->recording) {
                    
                    // Stop recording
                    this->recording = false;
                    
                    // Make a response message to the optimizer
                    auto result = std::make_unique<OptimizeScriptResult>();
                    
                    result->set_iteration(iteration);
                    result->set_metadata(metadata);
                    
                    // Store all the sensor values for the script
                    for(const auto& sensor : sensors) {
                        
                        auto* s = result->add_sensors();
                        
                        auto* acc = s->mutable_accelerometer();
                        acc->set_x(sensor->acceleronometer.x);
                        acc->set_y(sensor->acceleronometer.y);
                        acc->set_z(sensor->acceleronometer.z);
                        
                        auto* gyro = s->mutable_gyroscope();
                        gyro->set_x(sensor->gyroscope.x);
                        gyro->set_y(sensor->gyroscope.y);
                        gyro->set_z(sensor->gyroscope.z);
                        
                        auto* lfsr = s->mutable_leftfsr();
                        lfsr->set_fsr1(sensor->fsr.left.fsr1);
                        lfsr->set_fsr2(sensor->fsr.left.fsr2);
                        lfsr->set_fsr3(sensor->fsr.left.fsr3);
                        lfsr->set_fsr4(sensor->fsr.left.fsr4);
                        lfsr->set_centrex(sensor->fsr.left.centreX);
                        lfsr->set_centrey(sensor->fsr.left.centreY);
                        
                        auto* rfsr = s->mutable_rightfsr();
                        rfsr->set_fsr1(sensor->fsr.right.fsr1);
                        rfsr->set_fsr2(sensor->fsr.right.fsr2);
                        rfsr->set_fsr3(sensor->fsr.right.fsr3);
                        rfsr->set_fsr4(sensor->fsr.right.fsr4);
                        rfsr->set_centrex(sensor->fsr.right.centreX);
                        rfsr->set_centrey(sensor->fsr.right.centreY);
                        
                        for(int i = 0; i < 20; ++i) {
                            auto* servo = s->add_servos();
                            
                            servo->set_errorflags(sensor->servo[i].errorFlags);
                            servo->set_id(static_cast<messages::input::proto::Sensors_ServoID>(i));
                            servo->set_enabled(sensor->servo[i].torqueEnabled);
                            servo->set_pgain(sensor->servo[i].pGain);
                            servo->set_igain(sensor->servo[i].iGain);
                            servo->set_dgain(sensor->servo[i].dGain);
                            servo->set_goalposition(sensor->servo[i].goalPosition);
                            servo->set_movingspeed(sensor->servo[i].movingSpeed);
                            servo->set_torquelimit(sensor->servo[i].torqueLimit);
                            servo->set_presentposition(sensor->servo[i].presentPosition);
                            servo->set_presentspeed(sensor->servo[i].presentSpeed);
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
    }
}
