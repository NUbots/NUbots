/*
 * This file is part of ScriptEngine.
 *
 * ScriptEngine is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ScriptEngine is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with ScriptEngine.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#include "ScriptEngine.h"
#include "messages/Configuration.h"
#include "messages/ExecuteScript.h"
#include "messages/ServoWaypoint.h"

namespace messages {

    template<>
    struct ConfigurationNode::ConvertNode<modules::ScriptEngine::Script::Frame::Target> {

        static ConfigurationNode makeNode(const modules::ScriptEngine::Script::Frame::Target input) {

            ConfigurationNode node;

            node["id"] = messages::DarwinSensors::Servo::stringFromId(input.id);
            node["position"] = input.position;
            node["gain"] = input.gain;

            return node;
        }

        static modules::ScriptEngine::Script::Frame::Target makeValue(const ConfigurationNode& node) {

            return {
                messages::DarwinSensors::Servo::idFromString(node["id"]),
                static_cast<float>(node["position"]),
                static_cast<float>(node["gain"])
            };
        }
    };

    template<>
    struct ConfigurationNode::ConvertNode<modules::ScriptEngine::Script::Frame> {

        static ConfigurationNode makeNode(const modules::ScriptEngine::Script::Frame input) {

            ConfigurationNode node;

            node["duration"] = std::chrono::duration_cast<std::chrono::milliseconds>(input.duration).count();
            node["targets"] = input.targets;

            return node;
        }

        static modules::ScriptEngine::Script::Frame makeValue(const ConfigurationNode& node) {

            int millis = node["duration"];
            std::chrono::milliseconds duration(millis);

            std::vector<modules::ScriptEngine::Script::Frame::Target> targets = node["targets"];

            return {
                duration,
                std::move(targets)
            };
        }
    };

    template<>
    struct ConfigurationNode::ConvertNode<modules::ScriptEngine::Script> {

        static ConfigurationNode makeNode(const modules::ScriptEngine::Script input) {

            ConfigurationNode node;

            node = input.frames;

            return node;
        }

        static modules::ScriptEngine::Script makeValue(const ConfigurationNode& node) {

            std::vector<modules::ScriptEngine::Script::Frame> frames = node;

            return {
                std::move(frames)
            };
        }
    };
}

namespace modules {

    struct Scripts {
        // For scripts we want updates on the whole scripts directory
        static constexpr const char* CONFIGURATION_PATH = "scripts/";
    };

    ScriptEngine::ScriptEngine(NUClear::PowerPlant* plant) : Reactor(plant) {

        on<Trigger<messages::Configuration<Scripts>>>([this](const messages::Configuration<Scripts>& script) {

            // Add this script to our list of scripts
            // TODO get the actual name of the script
            scripts.insert(std::make_pair("some string", script.config));
        });

        on<Trigger<messages::ExecuteScript>>([this](const messages::ExecuteScript& command) {

            auto script = scripts.find(command.script);

            if(script == std::end(scripts)) {
                throw std::runtime_error("The script " + command.script + " is not loaded in the system");
            }
            else {

                // Wrap our raw pointer in a unique pointer, this way if an exception/error happens it will be deallocated
                auto waypoints = std::make_unique<std::vector<messages::ServoWaypoint>>();

                auto time = command.start;

                for(const auto& frame : script->second.frames) {
                    // Move along our duration in time
                    time += frame.duration;

                    // Loop through all the motors and make a servo waypoint for it
                    for(const auto& target : frame.targets) {
                        waypoints->push_back({
                            time,
                            target.id,
                            target.position,
                            target.gain
                        });
                    }
                }

                // Release from the unique pointer and emit it
                emit(std::move(waypoints));
            }
        });
    }
}
