#ifndef MESSAGES_CONFIGURATIONNODE_STRING_H
#define MESSAGES_CONFIGURATIONNODE_STRING_H

#include "../ConfigurationNode.h"

namespace Messages {
    template<>
    struct ConfigurationNode::ConvertNode<std::string> {

        static ConfigurationNode makeNode(const std::string input) {
            return ConfigurationNode(DataType::STRING, std::shared_ptr<std::string>(new std::string(input)));
        }

        static std::string makeValue(const ConfigurationNode& node) {
            switch(node.datatype) {
                case DataType::INTEGER:
                    std::to_string(*std::static_pointer_cast<int>(node.value));
                case DataType::FLOATINGPOINT:
                    std::to_string(*std::static_pointer_cast<double>(node.value));
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

#endif
