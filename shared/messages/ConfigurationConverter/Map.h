#ifndef MESSAGES_CONFIGURATIONNODE_MAP_H
#define MESSAGES_CONFIGURATIONNODE_MAP_H

#include "../ConfigurationNode.h"

namespace messages {
    template <>
    struct ConfigurationNode::ConvertNode<std::map<std::string, messages::ConfigurationNode>> {

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


#endif
