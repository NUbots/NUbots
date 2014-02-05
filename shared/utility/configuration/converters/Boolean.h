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

#ifndef UTILITY_CONFIGURATION_CONFIGURATIONNODE_BOOLEAN_H
#define UTILITY_CONFIGURATION_CONFIGURATIONNODE_BOOLEAN_H

#include "../ConfigurationNode.h"

namespace utility {
namespace configuration {
    template<>
    struct ConfigurationNode::ConvertNode<bool> {

        static ConfigurationNode makeNode(const bool input) {
            return ConfigurationNode(DataType::BOOLEAN, std::shared_ptr<bool>(new bool(input)));
        }

        static bool makeValue(const ConfigurationNode& node) {
            switch(node.datatype) {
                case DataType::INTEGER:
                    return (*std::static_pointer_cast<int>(node.value)) == 0 ? 0 : 1;
                case DataType::FLOATINGPOINT:
                    return (*std::static_pointer_cast<double>(node.value)) == 0 ? 0 : 1;
                case DataType::BOOLEAN:
                    return *std::static_pointer_cast<bool>(node.value);
                case DataType::STRING:
                    return !std::static_pointer_cast<std::string>(node.value)->empty();
                case DataType::ARRAY:
                case DataType::OBJECT:
                    return true;
                case DataType::NULLPOINTER:
                    return false;
                default:
                    return false;
            }
        }
    };
}
}

#endif
