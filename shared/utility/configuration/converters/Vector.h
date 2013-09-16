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

#ifndef UTILITY_CONFIGURATION_CONFIGURATIONNODE_VECTOR_H
#define UTILITY_CONFIGURATION_CONFIGURATIONNODE_VECTOR_H

#include "../ConfigurationNode.h"

namespace utility {
namespace configuration {
    template <>
    struct ConfigurationNode::ConvertNode<std::vector<ConfigurationNode>> {

        static ConfigurationNode makeNode(const std::vector<ConfigurationNode>& input) {
            return ConfigurationNode(DataType::ARRAY, std::shared_ptr<std::vector<ConfigurationNode>>(new std::vector<ConfigurationNode>(std::move(input))));
        }

        static std::vector<ConfigurationNode> makeValue(const ConfigurationNode& node) {
            switch (node.datatype) {
                case DataType::ARRAY:
                    return *std::static_pointer_cast<std::vector<ConfigurationNode>>(node.value);

                default:
                    throw std::runtime_error("The datatype in this node was not an array");
            }
        }
    };

    template <typename TType>
    struct ConfigurationNode::ConvertNode<std::vector<TType>> {

        static ConfigurationNode makeNode(const std::vector<TType>& input) {

            std::vector<ConfigurationNode> node;

            for (auto v : input) {
                node.push_back(v);
            }

            return node;
        }

        static std::vector<TType> makeValue(const ConfigurationNode& node) {
            switch (node.datatype) {
                case DataType::ARRAY: {

                    std::vector<TType> result;
                    for (auto& value : *std::static_pointer_cast<std::vector<ConfigurationNode>>(node.value)) {
                        result.push_back(value);
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
