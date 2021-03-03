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

#include <fmt/format.h>
#include <map>
#include <nuclear>
#include <string>
#include <yaml-cpp/yaml.h>

namespace utility::support {

struct LogLevel {
    LogLevel() {}
    LogLevel(const YAML::Node& node) : node(node) {}
    LogLevel(const LogLevel& other) {
        this->node = other.node;
    }

    operator ::NUClear::LogLevel() {
        ::NUClear::LogLevel value;

        try {
            // clang-format off
            auto lvl = node.as<std::string>();
            if (lvl == "TRACE") { value = NUClear::TRACE; }
            else if (lvl == "DEBUG") { value = NUClear::DEBUG; }
            else if (lvl == "INFO") { value = NUClear::INFO; }
            else if (lvl == "WARN") { value = NUClear::WARN; }
            else if (lvl == "ERROR") { value = NUClear::ERROR; }
            else if (lvl == "FATAL") { value = NUClear::FATAL; }
            // clang-format on
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
