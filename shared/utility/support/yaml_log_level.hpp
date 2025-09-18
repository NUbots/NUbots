/*
 * MIT License
 *
 * Copyright (c) 2021 NUbots
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

#ifndef UTILITY_SUPPORT_YAML_LOG_LEVEL_HPP
#define UTILITY_SUPPORT_YAML_LOG_LEVEL_HPP

#include <fmt/format.h>
#include <map>
#include <nuclear>
#include <string>
#include <yaml-cpp/yaml.h>

namespace YAML {

    /// @brief Converts between YAML::Node and NUClear::LogLevel
    /// @example this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
    template <>
    struct convert<::NUClear::LogLevel> {
        /// @brief Converts a NUClear::LogLevel to a YAML::Node that stores the name of the log level
        static Node encode(const ::NUClear::LogLevel& rhs) {
            Node node;

            switch (rhs) {
                case NUClear::LogLevel::TRACE: node = "TRACE"; break;
                case NUClear::LogLevel::DEBUG: node = "DEBUG"; break;
                case NUClear::LogLevel::INFO: node = "INFO"; break;
                case NUClear::LogLevel::WARN: node = "WARN"; break;
                case NUClear::LogLevel::ERROR: node = "ERROR"; break;
                case NUClear::LogLevel::FATAL: node = "FATAL"; break;
                default: node = "DEBUG"; break;
            }

            return node;
        }

        /// @brief Converts a YAML::Node that stores the name of a log level to the corresponding NUClear::LogLevel
        static bool decode(const Node& node, ::NUClear::LogLevel& rhs) {
            try {
                // clang-format off
                auto lvl = node.as<std::string>();
                if      (lvl == "TRACE")       { rhs = NUClear::LogLevel::TRACE; }
                else if (lvl == "INFO")        { rhs = NUClear::LogLevel::INFO;  }
                else if (lvl == "WARN")        { rhs = NUClear::LogLevel::WARN;  }
                else if (lvl == "ERROR")       { rhs = NUClear::LogLevel::ERROR; }
                else if (lvl == "FATAL")       { rhs = NUClear::LogLevel::FATAL; }
                else /* or (lvl == "DEBUG") */ { rhs = NUClear::LogLevel::DEBUG; }
                // clang-format on
            }
            catch (const std::invalid_argument& ex) {
                return false;
            }

            return true;
        }
    };
}  // namespace YAML

#endif  // UTILITY_SUPPORT_YAML_LOG_LEVEL_HPP
