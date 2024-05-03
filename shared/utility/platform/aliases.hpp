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

#ifndef UTILITY_PLATFORM_ALIASES_HPP
#define UTILITY_PLATFORM_ALIASES_HPP

#include <string>

namespace utility::platform {

    /// @brief Finds the matching alias for the hostname in the robot_names.yaml file, if it exists
    /// @param hostname The hostname of the robot
    /// @return The alias of the robot, if it exists, otherwise an empty string
    inline std::string get_robot_alias(std::string hostname) {
        // Get robot aliases
        YAML::Node config = YAML::LoadFile("config/robot_names.yaml");

        // Create a map to hold the aliases
        std::map<std::string, std::string> robot_aliases{};

        // Get the "robot_alias" node and iterate over it
        for (const auto& node : config["robot_alias"]) {
            // Get the hostname and name from each node and add them to the map
            robot_aliases[node["hostname"].as<std::string>()] = node["robot_name"].as<std::string>();
        }

        // Return the name of the robot or empty if it doesn't exist
        return robot_aliases.find(hostname) != robot_aliases.end() ? robot_aliases[hostname] : "";
    }
}  // namespace utility::platform

#endif  // UTILITY_PLATFORM_ALIASES_HPP
