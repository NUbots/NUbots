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

#ifndef UTILITY_SUPPORT_yaml_expression_H
#define UTILITY_SUPPORT_yaml_expression_H

#include <yaml-cpp/yaml.h>
#include <limits>

#include "math_string.h"

namespace utility {
namespace support {
    /**
     * Represents a mathematical expression
     * Acts as a double
     */
    struct Expression {
        double value;
        Expression() : value(0.0) {}
        Expression(double x) : value(x) {}
        operator double() const {
            return value;
        }
    };
}  // namespace support
}  // namespace utility

namespace YAML {

template <>
struct convert<utility::support::Expression> {
    static Node encode(const utility::support::Expression& rhs) {
        Node node;

        // Treat as a double
        node = rhs;

        return node;
    }

    static bool decode(const Node& node, utility::support::Expression& rhs) {

        // Parse the node as a string into a math expression
        rhs = utility::support::parse_math_string(node.as<std::string>());
        return true;
    }
};
}  // namespace YAML

#endif
