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

#ifndef UTILITY_CONFIGURATION_CONFIGURATIONNODE_MAP_H
#define UTILITY_CONFIGURATION_CONFIGURATIONNODE_MAP_H

#include "../ConfigurationNode.h"

namespace utility {
namespace configuration {
    template <>
    struct ConfigurationNode::ConvertNode<std::map<std::string, ConfigurationNode>> {

        static ConfigurationNode makeNode(const std::map<std::string, ConfigurationNode>& input) {
            return ConfigurationNode(DataType::OBJECT, std::shared_ptr<std::map<std::string, ConfigurationNode>>(new std::map<std::string, ConfigurationNode>(std::move(input))));
        }

        static std::map<std::string, ConfigurationNode> makeValue(const ConfigurationNode& node) {
            switch (node.datatype) {
                case DataType::OBJECT:
                    return *std::static_pointer_cast<std::map<std::string, ConfigurationNode>>(node.value);

                default:
                    throw std::runtime_error("The datatype in this node was not an object");
            }
        }
    };

    template <typename TType>
    struct ConfigurationNode::ConvertNode<std::map<std::string, TType>> {

        static ConfigurationNode makeNode(const std::map<std::string, TType>& input) {

            std::map<std::string, ConfigurationNode> node;

            for (auto v : input) {
                node.insert(v);
            }

            return node;
        }

        static std::map<std::string, TType> makeValue(const ConfigurationNode& node) {
            switch (node.datatype) {
                case DataType::OBJECT: {

                    std::map<std::string, TType> result;
                    for (auto& value : *std::static_pointer_cast<std::map<std::string, ConfigurationNode>>(node.value)) {
                        result.insert(value);
                    }
                    return result;
                }
                default:
                    throw std::runtime_error("The datatype in this node was not an array or an object");
            }
        }
    };
}
}

#endif
