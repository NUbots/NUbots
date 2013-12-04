/*
 * This file is part of ScriptEngine.
 *
 * ScriptEngine is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ScriptEngine is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ScriptEngine.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MESSAGES_SCRIPT_H
#define MESSAGES_SCRIPT_H

#include <chrono>
#include "utility/configuration/ConfigurationNode.h"
#include "messages/input/ServoID.h"

namespace messages {
    namespace motion {

        /**
         * TODO document
         *
         * @author Trent Houliston
         */
        struct Script {
            struct Frame {
                struct Target {
                    input::ServoID id;
                    float position;
                    float gain;
                };
                NUClear::clock::duration duration;
                std::vector<Target> targets;
            };
            std::vector<Frame> frames;
        };

        /**
         * TODO document
         *
         * @author Trent Houliston
         */
        struct ExecuteScriptByName {
            ExecuteScriptByName(const std::string script, NUClear::clock::time_point start = NUClear::clock::now()) : script(script), start(start) {};
            std::string script;
            NUClear::clock::time_point start;
        };

        /**
         * TODO document
         *
         * @author Trent Houliston
         */
        struct ExecuteScript {
            ExecuteScript(const Script script, NUClear::clock::time_point start = NUClear::clock::now()) : script(script), start(start) {};
            Script script;
            NUClear::clock::time_point start;
        };
        
    }  // motion
}  // messages

namespace utility {
namespace configuration {

    /**
     * TODO document
     *
     * @author Trent Houliston
     */
    template<>
    struct utility::configuration::ConfigurationNode::ConvertNode<messages::motion::Script::Frame::Target> {

        static utility::configuration::ConfigurationNode makeNode(const messages::motion::Script::Frame::Target input) {

            utility::configuration::ConfigurationNode node;

            node["id"] = messages::input::stringFromId(input.id);
            node["position"] = input.position;
            node["gain"] = input.gain;

            return node;
        }

        static messages::motion::Script::Frame::Target makeValue(const utility::configuration::ConfigurationNode& node) {

            return
            {
                messages::input::idFromString(node["id"]),
                        static_cast<float>(node["position"]),
                        static_cast<float>(node["gain"])
            };
        }
    };

    /**
     * TODO document
     *
     * @author Trent Houliston
     */
    template<>
    struct utility::configuration::ConfigurationNode::ConvertNode<messages::motion::Script::Frame> {

        static utility::configuration::ConfigurationNode makeNode(const messages::motion::Script::Frame input) {

            utility::configuration::ConfigurationNode node;

            node["duration"] = std::chrono::duration_cast<std::chrono::milliseconds>(input.duration).count();
            node["targets"] = input.targets;

            return node;
        }

        static messages::motion::Script::Frame makeValue(const utility::configuration::ConfigurationNode& node) {

            int millis = node["duration"];
            std::chrono::milliseconds duration(millis);

            std::vector<messages::motion::Script::Frame::Target> targets = node["targets"];

            return {duration, std::move(targets)};
        }
    };

    /**
     * TODO document
     *
     * @author Trent Houliston
     */
    template<>
    struct utility::configuration::ConfigurationNode::ConvertNode<messages::motion::Script> {

        static utility::configuration::ConfigurationNode makeNode(const messages::motion::Script input) {
            utility::configuration::ConfigurationNode node;
            node = input.frames;
            return node;
        }

        static messages::motion::Script makeValue(const utility::configuration::ConfigurationNode& node) {
            std::vector<messages::motion::Script::Frame> frames = node;
            return {std::move(frames)};
        }
    };
}  // namespace configuration
}  // namespace utility

#endif
