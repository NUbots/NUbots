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
#include <iostream>
#include <limits>
#include "exprtk.hpp"

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

        // Add constants to the symbol table
        exprtk::symbol_table<double> table;
        table.add_constants();
        table.add_constant("auto", std::numeric_limits<double>::infinity());

        // Add table to expression
        exprtk::expression<double> expression;
        expression.register_symbol_table(table);

        // Add expression to parser and parse
        exprtk::parser<double> parser;
        parser.compile(node.as<std::string>(), expression);

        rhs = expression.value();
        return true;
    }
};
}  // namespace YAML

#endif
