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
            }
        }
    };
}
}

#endif
