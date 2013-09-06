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
#include "ConfigurationNode.h"
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

    // These convert Script objects into ConfigurationNode objects and visa versa

    template<>
    struct ConfigurationNode::ConvertNode<Script::Frame::Target> {

        static ConfigurationNode makeNode(const Script::Frame::Target input) {

            ConfigurationNode node;

            node["id"] = messages::DarwinSensors::Servo::stringFromId(input.id);
            node["position"] = input.position;
            node["gain"] = input.gain;

            return node;
        }

        static Script::Frame::Target makeValue(const ConfigurationNode& node) {

            return
            {
                messages::DarwinSensors::Servo::idFromString(node["id"]),
                        static_cast<float> (node["position"]),
                        static_cast<float> (node["gain"])
            };
        }
    };

    template<>
    struct ConfigurationNode::ConvertNode<Script::Frame> {

        static ConfigurationNode makeNode(const Script::Frame input) {

            ConfigurationNode node;

            node["duration"] = std::chrono::duration_cast<std::chrono::milliseconds>(input.duration).count();
            node["targets"] = input.targets;

            return node;
        }

        static Script::Frame makeValue(const ConfigurationNode& node) {

            int millis = node["duration"];
            std::chrono::milliseconds duration(millis);

            std::vector<Script::Frame::Target> targets = node["targets"];

            return {duration, std::move(targets)};
        }
    };

    template<>
    struct ConfigurationNode::ConvertNode<Script> {

        static ConfigurationNode makeNode(const Script input) {
            ConfigurationNode node;
            node = input.frames;
            return node;
        }

        static Script makeValue(const ConfigurationNode& node) {
            std::vector<Script::Frame> frames = node;
            return {std::move(frames)};
        }
    };
};

#endif