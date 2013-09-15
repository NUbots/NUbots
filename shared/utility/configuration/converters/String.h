/*
 * This file is part of ConfigurationNode.
 *
 * ConfigurationNode is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ConfigurationNode is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ConfigurationNode.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#ifndef UTILITY_CONFIGURATION_CONFIGURATIONNODE_STRING_H
#define UTILITY_CONFIGURATION_CONFIGURATIONNODE_STRING_H

#include "../ConfigurationNode.h"

namespace utility {
namespace configuration {
    template<>
    struct ConfigurationNode::ConvertNode<std::string> {

        static ConfigurationNode makeNode(const std::string input) {
            return ConfigurationNode(DataType::STRING, std::shared_ptr<std::string>(new std::string(input)));
        }

        static std::string makeValue(const ConfigurationNode& node) {
            switch(node.datatype) {
                case DataType::INTEGER:
                    return std::to_string(*std::static_pointer_cast<int>(node.value));
                case DataType::FLOATINGPOINT:
                    return std::to_string(*std::static_pointer_cast<double>(node.value));
                case DataType::BOOLEAN:
                    return *std::static_pointer_cast<bool>(node.value) ? "true" : "false";
                case DataType::STRING:
                    return *std::static_pointer_cast<std::string>(node.value);
                default:
                    throw std::runtime_error("The datatype in this node could not be converted to a string");

            }
        }
    };

    template<>
    struct ConfigurationNode::ConvertNode<const char*> {

        static ConfigurationNode makeNode(const char* input) {
            return ConvertNode<std::string>::makeNode(std::string(input));
        }

        static const char* makeValue(const ConfigurationNode& node) {
            return ConvertNode<std::string>::makeValue(node).c_str();
        }
    };
}
}
#endif
