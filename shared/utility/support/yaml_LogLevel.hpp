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

#ifndef UTILITY_SUPPORT_yaml_LogLevel_HPP
#define UTILITY_SUPPORT_yaml_LogLevel_HPP

#include <map>
#include <nuclear>
#include <string>
#include <yaml-cpp/yaml.h>

namespace utility::support {

struct LogLevel {

    // A map so we can move from strings returned by YAML to our enum.
    constexpr std::map<std::string, ::NUClear::LogLevel> string_to_LogLevel{{"TRACE", NUClear::TRACE},
                                                                            {"DEBUG", NUClear::DEBUG},
                                                                            {"INFO", NUClear::INFO},
                                                                            {"WARN", NUClear::WARN},
                                                                            {"ERROR", NUClear::ERROR},
                                                                            {"FATAL", NUClear::FATAL}};

    LogLevel() {}
    LogLevel(const YAML::Node& node) : node(node) {}

    operator ::NUClear::LogLevel() {
        ::NUClear::LogLevel value;

        try {
            value = string_to_LogLevel.at(ode.as<std::string>());
        }

        catch (const std::invalid_argument& ex) {
            throw std::invalid_argument(
                fmt::format("Unable to convert node to NUClear::LogLevel type.\n{}", ex.what()));
        }

        return value;
    }

private:
    YAML::Node node;
};
}  // namespace utility::support

namespace YAML {

template <>
struct convert<utility::support::LogLevel> {
    static Node encode(const utility::support::LogLevel& rhs) {
        Node node;

        // Treat as a int???
        node = rhs;

        return node;
    }

    static bool decode(const Node& node, utility::support::LogLevel& rhs) {
        rhs = utility::support::LogLevel(node);
        return true;
    }
};
}  // namespace YAML

#endif  // UTILITY_SUPPORT_yaml_LogLevel_HPP
