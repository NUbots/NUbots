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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_SUPPORT_YAMLEXPRESSION_H
#define UTILITY_SUPPORT_YAMLEXPRESSION_H

#include <iostream>
#include <yaml-cpp/yaml.h>

extern "C" {
    #include <matheval.h>
}


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
        operator double() const { return value; }
    };
}
}

namespace YAML {

    template<>
    struct convert<utility::support::Expression> {
        static Node encode(const utility::support::Expression& rhs) {
            Node node;

            // Treat as a double
            node = rhs;

            return node;
        }

        static bool decode(const Node& node, utility::support::Expression& rhs) {

            // Cast the node's value to a non-const C-style string
            // This is as the evaluator_create function expects this type
            auto expression_string = const_cast<char*>(node.as<std::string>().c_str());

            // This horrible line constructs an expression from the expression_string
            // This is so that the evaluator_destroy function will automatically be called on deallocation
            auto expression = std::unique_ptr<void, std::function<void(void*)>>(evaluator_create(expression_string), evaluator_destroy);

            // Evaluate the expression
            // TODO: support variables
            rhs = evaluator_evaluate(expression.get(), 0, nullptr, nullptr);

            return true;
        }
    };

}

#endif
