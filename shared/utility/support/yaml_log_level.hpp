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

namespace YAML {

template <>
struct convert<::NUclear::LogLevel> {
    static Node encode(const ::NUclear::LogLevel& rhs) {
        Node node;


        // Treat as a int???
        node = rhs;

        return node;
    }

    static bool decode(const Node& node, ::NUclear::LogLevel& rhs) {
        try {
            // clang-format off
            auto lvl = node.as<std::string>();
            if (lvl == "TRACE") { rhs = NUClear::TRACE; }
            else if (lvl == "DEBUG") { rhs = NUClear::DEBUG; }
            else if (lvl == "INFO") { rhs = NUClear::INFO; }
            else if (lvl == "WARN") { rhs = NUClear::WARN; }
            else if (lvl == "ERROR") { rhs = NUClear::ERROR; }
            else if (lvl == "FATAL") { rhs = NUClear::FATAL; }
            // clang-format on
        }
        catch (const std::invalid_argument& ex) {
            return false;
        }

        return true;
    }
};
}  // namespace YAML

#endif  // UTILITY_SUPPORT_yaml_LogLevel_HPP
