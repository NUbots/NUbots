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

#ifndef MESSAGES_SCRIPT_H
#define MESSAGES_SCRIPT_H

#include <chrono>
#include "utility/configuration/ConfigurationNode.h"
#include "DarwinSensors.h"

namespace messages {

    struct Script {
        struct Frame {
            struct Target {
                messages::DarwinSensors::Servo::ID id;
                float position;
                float gain;
            };
            NUClear::clock::duration duration;
            std::vector<Target> targets;
        };
        std::vector<Frame> frames;
    };
} // \namespace messages

namespace utility {
namespace configuration {

    // These convert Script objects into utility::configuration::ConfigurationNode objects and visa versa

    template<>
    struct utility::configuration::ConfigurationNode::ConvertNode<messages::Script::Frame::Target> {

        static utility::configuration::ConfigurationNode makeNode(const messages::Script::Frame::Target input) {

            utility::configuration::ConfigurationNode node;

            node["id"] = messages::DarwinSensors::Servo::stringFromId(input.id);
            node["position"] = input.position;
            node["gain"] = input.gain;

            return node;
        }

        static messages::Script::Frame::Target makeValue(const utility::configuration::ConfigurationNode& node) {

            return
            {
                messages::DarwinSensors::Servo::idFromString(node["id"]),
                        static_cast<float> (node["position"]),
                        static_cast<float> (node["gain"])
            };
        }
    };

    template<>
    struct utility::configuration::ConfigurationNode::ConvertNode<messages::Script::Frame> {

        static utility::configuration::ConfigurationNode makeNode(const messages::Script::Frame input) {

            utility::configuration::ConfigurationNode node;

            node["duration"] = std::chrono::duration_cast<std::chrono::milliseconds>(input.duration).count();
            node["targets"] = input.targets;

            return node;
        }

        static messages::Script::Frame makeValue(const utility::configuration::ConfigurationNode& node) {

            int millis = node["duration"];
            std::chrono::milliseconds duration(millis);

            std::vector<messages::Script::Frame::Target> targets = node["targets"];

            return {duration, std::move(targets)};
        }
    };

    template<>
    struct utility::configuration::ConfigurationNode::ConvertNode<messages::Script> {

        static utility::configuration::ConfigurationNode makeNode(const messages::Script input) {
            utility::configuration::ConfigurationNode node;
            node = input.frames;
            return node;
        }

        static messages::Script makeValue(const utility::configuration::ConfigurationNode& node) {
            std::vector<messages::Script::Frame> frames = node;
            return {std::move(frames)};
        }
    };
} // \namespace configuration
} // \namespace utility

#endif
