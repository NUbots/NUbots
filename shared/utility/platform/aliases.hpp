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
        try {
            // Get robot aliases
            YAML::Node config = YAML::LoadFile("config/robot_names.yaml");

            // Return an alias if it matches the hostname
            for (const auto& node : config["robot_alias"]) {
                if (node["hostname"].as<std::string>() == hostname) {
                    return node["name"].as<std::string>();
                }
            }
            return "";
        }
        // If anything went wrong, return empty string
        catch (const std::exception& e) {
            return "";
        }
    }
}  // namespace utility::platform

#endif  // UTILITY_PLATFORM_ALIASES_HPP
