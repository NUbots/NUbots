#ifndef UTILITY_CONFIGURATION_CONFIGURATIONNODE_NUMBER_H
#define UTILITY_CONFIGURATION_CONFIGURATIONNODE_NUMBER_H

#include "../ConfigurationNode.h"

namespace utility {
namespace configuration {
    template <typename TType>
    struct ConfigurationNode::ConvertNode {

        // FOR INTEGERS

        /**
         *
         */
        template <typename T = TType>
        static ConfigurationNode makeNode(const typename std::enable_if<std::is_integral<T>::value, TType>::type input) {
            return ConfigurationNode(DataType::INTEGER, std::shared_ptr<long>(new long(input)));
        }

        /**
         *
         */
        template <typename T = TType>
        static typename std::enable_if<std::is_integral<T>::value, TType>::type makeValue(const ConfigurationNode& node) {
            switch(node.datatype) {
                case DataType::INTEGER:
                    return static_cast<TType>(*std::static_pointer_cast<long>(node.value));
                case DataType::FLOATINGPOINT:
                    return static_cast<TType>(*std::static_pointer_cast<double>(node.value));
                case DataType::BOOLEAN:
                    return static_cast<TType>((*std::static_pointer_cast<bool>(node.value)) ? 1 : 0);
                case DataType::STRING:
                    return static_cast<TType>(atof(std::static_pointer_cast<std::string>(node.value)->c_str()));
                default:
                    throw std::runtime_error("The datatype in this node could not be converted to an integer");
            }
        }

        // FOR FLOATING POINT

        /**
         *
         */
        template <typename T = TType>
        static ConfigurationNode makeNode(const typename std::enable_if<std::is_floating_point<T>::value, TType>::type input) {
            return ConfigurationNode(DataType::FLOATINGPOINT, std::shared_ptr<double>(new double(input)));
        }

        /**
         *
         */
        template <typename T = TType>
        static typename std::enable_if<std::is_floating_point<T>::value, TType>::type makeValue(const ConfigurationNode& node) {
            switch(node.datatype) {
                case DataType::INTEGER:
                    return *std::static_pointer_cast<int>(node.value);
                case DataType::FLOATINGPOINT:
                    return *std::static_pointer_cast<double>(node.value);
                case DataType::BOOLEAN:
                    return (*std::static_pointer_cast<bool>(node.value)) ? 1 : 0;
                case DataType::STRING:
                    return atof(std::static_pointer_cast<std::string>(node.value)->c_str());
                default:
                    throw std::runtime_error("The datatype in this node could not be converted to a double");
            }
        }
    };
}
}

#endif
