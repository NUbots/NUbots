#ifndef MESSAGES_CONFIGURATIONNODE_H
#define MESSAGES_CONFIGURATIONNODE_H

#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <map>

namespace Messages {
    class ConfigurationNode {
    private:
        enum class DataType {
            STRING,
            INTEGER,
            FLOATINGPOINT,
            BOOLEAN,
            ARRAY,
            OBJECT,
            NULLPOINTER
        } datatype;

        std::shared_ptr<void> value;

        template <typename TType>
        struct ConvertNode;

        ConfigurationNode(DataType datatype, std::shared_ptr<void> value) : datatype(datatype), value(value) {
        }

    public:

        ConfigurationNode() : datatype(DataType::NULLPOINTER) {};

        template <typename TType>
        ConfigurationNode(TType source) {
            *this = std::move(ConfigurationNode::ConvertNode<TType>::makeNode(source));
        }

        template <typename TType>
        void add(TType input) {
            if (datatype == DataType::NULLPOINTER) {
                datatype = DataType::ARRAY;
                value = std::shared_ptr<std::vector<ConfigurationNode>>(new std::vector<ConfigurationNode>());
            }

            if (datatype == DataType::ARRAY) {
                std::static_pointer_cast<std::vector<ConfigurationNode>>(value)->push_back(input);
            }
            else {
                throw std::runtime_error("The datatype in this node was not an array or nullptr");
            }
        }

        template <typename TType>
        void add(std::string key, TType input) {
            if (datatype == DataType::NULLPOINTER) {
                datatype = DataType::OBJECT;
                value = std::shared_ptr<std::map<std::string, ConfigurationNode>>(new std::map<std::string, ConfigurationNode>());
            }

            if (datatype == DataType::OBJECT) {
                std::static_pointer_cast<std::map<std::string, ConfigurationNode>>(value)->insert(std::make_pair(key, input));
            }
            else {
                throw std::runtime_error("The datatype in this node was not an object or nullptr");
            }
        }

        // TODO add the iterator functions

        // TODO add the stream operators

        ConfigurationNode& operator [] (std::string key) {

            if (datatype == DataType::NULLPOINTER) {
                datatype = DataType::OBJECT;
                value = std::shared_ptr<std::map<std::string, ConfigurationNode>>(new std::map<std::string, ConfigurationNode>());
            }

            if (datatype == DataType::OBJECT) {
                return (*std::static_pointer_cast<std::map<std::string, ConfigurationNode>>(value))[key];
            }
            else {
                std::stringstream error;
                error << "The datatype in this node was not an object. Found: " << (int)datatype;
                throw std::runtime_error(error.str());
            }
        }

        const ConfigurationNode& operator [] (std::string key) const {

            if (datatype == DataType::OBJECT) {
                return (*std::static_pointer_cast<std::map<std::string, ConfigurationNode>>(value))[key];
            }
            else {
                throw std::runtime_error("The datatype in this node was not an object");
            }
        }

        ConfigurationNode& operator [] (const char* key) {
            return operator[](std::string(key));
        }

        const ConfigurationNode& operator [] (const char* key) const {
            return operator[](std::string(key));
        }

        ConfigurationNode& operator [] (size_t index) {

            if (datatype == DataType::ARRAY) {
                return std::static_pointer_cast<std::vector<ConfigurationNode>>(value)->at(index);
            }
            else {
                throw std::runtime_error("The datatype in this node was not an array");
            }
        }

        const ConfigurationNode& operator [] (size_t index) const {

            if (datatype == DataType::ARRAY) {
                return std::static_pointer_cast<std::vector<ConfigurationNode>>(value)->at(index);
            }
            else {
                throw std::runtime_error("The datatype in this node was not an array");
            }
        }
        
        template <typename TType>
        operator TType() const {
            return static_cast<TType>(ConvertNode<TType>::makeValue(*this));
        }
    };
}

#include "ConfigurationConverter/String.h"
#include "ConfigurationConverter/Number.h"
#include "ConfigurationConverter/Boolean.h"
#include "ConfigurationConverter/Vector.h"
#include "ConfigurationConverter/Map.h"

#endif
