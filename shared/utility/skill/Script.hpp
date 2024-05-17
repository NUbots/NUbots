/*
 * MIT License
 *
 * Copyright (c) 2022 NUbots
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
#ifndef UTILITY_SKILL_SCRIPT_HPP
#define UTILITY_SKILL_SCRIPT_HPP

#include <nuclear>
#include <regex>
#include <yaml-cpp/yaml.h>

#include "extension/Behaviour.hpp"

#include "message/actuation/ServoCommand.hpp"

#include "utility/file/fileutil.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/platform/aliases.hpp"
#include "utility/support/hostname.hpp"


/**
 * @author Ysobel Sims
 */

namespace utility::skill {

    using message::actuation::ServoCommand;
    using message::actuation::ServoState;
    using utility::input::ServoID;

    /// @brief One Script to run, with name of the script and a duration modifier to speed up or slow down the Script
    struct ScriptRequest {
        std::string name;
        float duration_modifier{1.0};
    };

    // One frame of a script
    struct Frame {
        /// @brief Servo targets for each active servo in the frame
        struct Target {
            /// @brief The servo ID that this target is for; see ServoID.hpp
            ServoID id;
            /// @brief Target position for this servo in this frame
            float position{0.0f};
            /// @brief Gain for this servo in this frame
            float gain{0.0f};
            /// @brief Torque for this servo in this frame
            float torque{0.0f};
        };

        /// @brief The time it should take for the servos to move to their targets
        NUClear::clock::duration duration{};
        /// @brief Target position of servos for this frame
        std::vector<Target> targets;
    };

    template <typename Sequence>
    struct Script {
        std::vector<Frame> frames;
    };

    /// @brief Assumes the hostname is of the form <platform_name><robot_number> and returns <platform_name>.
    /// @param hostname The hostname of the device the program is running on.
    /// @return The platform name.
    /// @throws System error when the platform cannot be extracted from the hostname.
    static inline std::string get_platform(const std::string& hostname) {
        // It is assumed that all hostnames are in the format <platform name><robot number>,
        // such that the regular expression
        // [a-z]+[0-9]+?
        // will match all hostnames
        std::regex re("([a-z]+)([0-9]+)?");
        std::smatch match;

        if (std::regex_match(hostname, match, re)) {
            // match[0] will be the full string
            // match[1] the first match (platform name)
            // match[2] the second match (robot number)
            return match[1].str();
        }

        throw std::system_error(-1,
                                std::system_category(),
                                ("Failed to extract platform name from '" + hostname + "'."));
    }

    /// @brief If a robot-specific script exists, load the YAML file as a script. If not, loads the
    /// platform-specific script. If neither exist, throws a runtime error.
    /// @param script The name of the script to run.
    /// @return The script contents in the form off a YAML node
    /// @throws Runtime error when the script doesn't exist.
    static YAML::Node load(const std::string& script) {
        // Set paths to the script files.
        auto hostname      = utility::support::get_hostname();
        auto robot_path    = "scripts/" + hostname + "/" + script;
        auto platform_path = "scripts/" + get_platform(hostname) + "/" + script;
        auto robot_name    = utility::platform::get_robot_alias(hostname);
        auto name_path     = "scripts/" + robot_name + "/" + script;

        // If robot name script exists, use it
        if (!robot_name.empty() && utility::file::exists(name_path)) {
            NUClear::log<NUClear::INFO>("Parsing name specific script:", script);
            return YAML::LoadFile(name_path);
        }
        // If robot name script doesn't exist, try to use the robot-specific script
        else if (utility::file::exists(robot_path)) {
            NUClear::log<NUClear::INFO>("Parsing robot specific script:", script);
            return YAML::LoadFile(robot_path);
        }
        // If there was no robot-specific script, then get the platform-specific script
        else if (utility::file::exists(platform_path)) {
            NUClear::log<NUClear::INFO>("Parsing default platform script:", script);
            return YAML::LoadFile(platform_path);
        }
        // The script doesn't exist, tell the user
        else {
            throw std::runtime_error("Script file '" + script + "' does not exist.");
        }
    }

    /// @brief Creates sequences of servos from the script requests given
    /// @param scripts Sequence of scripts to run
    /// @param start When the first script should start executing. Default is now.
    /// @return Sequence message which will be filled with sequences of servos and can then be emitted as a
    /// Task.
    template <typename Sequence>
    static std::unique_ptr<Sequence> load_script(const std::vector<ScriptRequest> scripts,
                                                 NUClear::clock::time_point start = NUClear::clock::now()) {
        // Create sequence message
        auto msg = std::make_unique<Sequence>();

        // First script begins at start time
        auto time = start;

        // Loop through all given scripts and add to the Sequence message
        for (const auto& script : scripts) {
            // Load the script to a vector of Frame structs
            auto frames = load(script.name).as<std::vector<Frame>>();
            // Time will be incremented through each frame
            // Loop over the frames and add them as sequences of servos into the Sequence message
            for (const auto& frame : frames) {
                // This frame should finish after frame.duration time has passed
                // A duraction modifier can be used to change the speed of the script
                time += std::chrono::duration_cast<NUClear::clock::time_point::duration>(frame.duration
                                                                                         * script.duration_modifier);

                // Add the servos in the frame to a map
                std::map<uint32_t, ServoCommand> servos{};
                for (const auto& target : frame.targets) {
                    servos[target.id] = ServoCommand(time, target.position, ServoState(target.gain, target.torque));
                }

                // Add the map to the pack. This represents one sequence of servos.
                msg->frames.emplace_back(servos);
            }
        }

        // Return the message which can then be emitted as a Task
        return msg;
    }

    /// @brief Emits a script function with one script request which is then sent through to the multi script version
    /// above
    /// @param script Script to run
    /// @param start When the script should start executing. Default is now.
    /// @return Sequence message which will be filled with sequences of servos.
    template <typename Sequence>
    static std::unique_ptr<Sequence> load_script(const ScriptRequest script,
                                                 NUClear::clock::time_point start = NUClear::clock::now()) {
        return load_script<Sequence>(std::vector<ScriptRequest>{script}, start);
    }

    /// @brief Accepts a vector of strings and converts them into a ScriptRequest, and uses the ScriptRequest
    /// function above
    /// @param scripts Script names to run
    /// @param start When the script should start executing. Default is now.
    /// @return Sequence message which will be filled with sequences of servos.
    template <typename Sequence>
    static std::unique_ptr<Sequence> load_script(const std::vector<std::string> scripts,
                                                 NUClear::clock::time_point start = NUClear::clock::now()) {
        // Convert the vector of strings into a vector of ScriptRequests
        std::vector<ScriptRequest> script_requests;
        for (const auto& script : scripts) {
            script_requests.emplace_back(script);
        }

        // Call the function above
        return load_script<Sequence>(script_requests, start);
    }

    /// @brief Accepts a string and converts it into a ScriptRequest, and uses the ScriptRequest function above
    /// @param script Script name to run
    /// @param start When the script should start executing. Default is now.
    /// @return Sequence message which will be filled with sequences of servos.
    template <typename Sequence>
    static std::unique_ptr<Sequence> load_script(const std::string script,
                                                 NUClear::clock::time_point start = NUClear::clock::now()) {
        return load_script<Sequence>(std::vector{ScriptRequest(script)}, start);
    }

    /// @brief Creates sequences of servos from the Script given
    /// @param script Script to load
    /// @param start When the first script should start executing. Default is now.
    /// @return Sequence message which will be filled with sequences of servos and can then be emitted as a
    /// Task.
    template <typename Sequence>
    static std::unique_ptr<Sequence> load_script(const Script<Sequence>& script,
                                                 NUClear::clock::time_point start = NUClear::clock::now()) {
        // Create sequence message
        auto msg = std::make_unique<Sequence>();

        // First script begins at start time
        auto time = start;

        // Time will be incremented through each frame
        // Loop over the frames and add them as sequences of servos into the Sequence message
        for (const auto& frame : script.frames) {
            // This frame should finish after frame.duration time has passed
            time += std::chrono::duration_cast<NUClear::clock::time_point::duration>(frame.duration);

            // Add the servos in the frame to a map
            std::map<uint32_t, ServoCommand> servos{};
            for (const auto& target : frame.targets) {
                servos[target.id] = ServoCommand(time, target.position, ServoState(target.gain, target.torque));
            }

            // Add the map to the pack. This represents one sequence of servos.
            msg->frames.emplace_back(servos);
        }

        // Return the message which can then be emitted as a Task
        return msg;
    }


}  // namespace utility::skill

// Functionality for reading in scripts
namespace YAML {
    template <>
    struct convert<::utility::skill::Frame::Target> {
        /// @brief Encodes a Target as a YAML Node
        /// @param rhs The Target to convert to a YAML Node
        /// @return A YAML Node of the Target
        static inline Node encode(const ::utility::skill::Frame::Target& rhs) {
            Node node;

            node["id"]       = static_cast<std::string>(rhs.id);
            node["position"] = rhs.position;
            node["gain"]     = rhs.gain;
            node["torque"]   = rhs.torque;

            return node;
        }
        /// @brief Decodes a YAML Node as a Target
        /// @param node The YAML Node to decode
        /// @param rhs A Target where the YAML Node information will be placed
        /// @return True if decoding was successful, false otherwise
        static inline bool decode(const Node& node, ::utility::skill::Frame::Target& rhs) {
            // Try to read and save the target information
            try {
                rhs = ::utility::skill::Frame::Target{node["id"].as<std::string>(),
                                                      node["position"].as<float>(),
                                                      node["gain"].as<float>(),
                                                      node["torque"].IsDefined() ? node["torque"].as<float>() : 100.0f};
            }
            catch (const YAML::Exception& e) {
                NUClear::log<NUClear::ERROR>("Error parsing script -",
                                             "Line:",
                                             e.mark.line,
                                             "Column:",
                                             e.mark.column,
                                             "Pos:",
                                             e.mark.pos,
                                             "Message:",
                                             e.msg);
                return false;
            }

            return true;
        }
    };

    template <>
    struct convert<::utility::skill::Frame> {
        /// @brief Encodes a Frame as a YAML Node
        /// @param rhs The Frame to convert to a YAML Node
        /// @return A YAML Node of the Frame
        static inline Node encode(const ::utility::skill::Frame& rhs) {
            Node node;

            node["duration"] = std::chrono::duration_cast<std::chrono::milliseconds>(rhs.duration).count();
            node["targets"]  = rhs.targets;

            return node;
        }

        /// @brief Decodes a YAML Node as a Frame
        /// @param node The YAML Node to decode
        /// @param rhs A Frame where the YAML Node information will be placed
        /// @return True if decoding was successful, false otherwise
        static inline bool decode(const Node& node, ::utility::skill::Frame& rhs) {
            try {
                int millis = node["duration"].as<int>();
                std::chrono::milliseconds duration(millis);

                auto targets = node["targets"].as<std::vector<::utility::skill::Frame::Target>>();
                rhs          = {duration, targets};
            }
            catch (const YAML::Exception& e) {
                NUClear::log<NUClear::ERROR>("Error parsing script -",
                                             "Line:",
                                             e.mark.line,
                                             "Column:",
                                             e.mark.column,
                                             "Pos:",
                                             e.mark.pos,
                                             "Message:",
                                             e.msg);
                return false;
            }

            return true;
        }
    };

    template <typename Sequence>
    struct convert<::utility::skill::Script<Sequence>> {
        /// @brief Encodes a Script as a YAML Node
        /// @param rhs The Script to convert to a YAML Node
        /// @return A YAML Node of the Script
        static inline Node encode(const ::utility::skill::Script<Sequence>& rhs) {
            Node node;

            node = rhs.frames;

            return node;
        }

        /// @brief Decodes a YAML Node as a Script
        /// @param node The YAML Node to decode
        /// @param rhs A Script where the YAML Node information will be placed
        /// @return True if decoding was successful, false otherwise
        static inline bool decode(const Node& node, ::utility::skill::Script<Sequence>& rhs) {
            try {
                auto frames = node.as<std::vector<::utility::skill::Frame>>();
                rhs         = {frames};
            }
            catch (const YAML::Exception& e) {
                NUClear::log<NUClear::ERROR>("Error parsing script -",
                                             "Line:",
                                             e.mark.line,
                                             "Column:",
                                             e.mark.column,
                                             "Pos:",
                                             e.mark.pos,
                                             "Message:",
                                             e.msg);
                return false;
            }

            return true;
        }
    };
}  // namespace YAML

#endif  // UTILITY_SKILL_SCRIPT_HPP
