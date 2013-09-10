#include "Serializer.h"

#include <vector>
#include <cctype>
#include <sstream>

using namespace utility;
using namespace configuration;
using namespace json;

namespace {
    class SerializerImpl {
        public:
            std::string serialize(const ConfigurationNode& root) {
                return serializeImpl(root);
            }
        private:
            std::string serializeImpl(const ConfigurationNode& node) {
                switch(node.nativeType()) {
                    case ConfigurationNode::DataType::STRING: 
                    case ConfigurationNode::DataType::INTEGER:
                    case ConfigurationNode::DataType::FLOATINGPOINT:
                    case ConfigurationNode::DataType::BOOLEAN:
                        return serializePrimative(node);
                        break;
                    case ConfigurationNode::DataType::NULLPOINTER:
                        return serializeNullpointer(node);
                        break;
                    case ConfigurationNode::DataType::ARRAY:
                        return serializeArray(node);
                        break;
                    case ConfigurationNode::DataType::OBJECT:
                        return serializeObject(node);
                        break;
                }

                // TODO: Add extra info here
                throw std::runtime_error(std::string("Bad node found: "));
            }

            std::string serializePrimative(const ConfigurationNode& node) {
                
                std::string nodeAsString = node;

                // Serialize the value
                if(node.nativeType() == ConfigurationNode::DataType::STRING) {
                    nodeAsString = "\"" + nodeAsString + "\"";
                }

                return nodeAsString;
            }

            std::string serializeNullpointer(const ConfigurationNode& node) {
                return "null";
            }

            std::string serializeArray(const ConfigurationNode& node) {
                std::string nodeAsString = "[";
                std::vector<ConfigurationNode> children = node;

                // Combine all the children into a serialized comma-separated list.
                std::string sep = "";
                for(auto& child : children) {
                    nodeAsString += sep;
                    nodeAsString += serializeImpl(child);
                    sep = ", ";
                }

                return nodeAsString + "]";
            }

            std::string serializeObject(const ConfigurationNode& node) {
                std::string nodeAsString = "{";
                std::map<std::string, ConfigurationNode> children = node;

                std::string sep = "";
                for(auto pair : children) {
                    nodeAsString += sep;
                    nodeAsString += "\"" + pair.first + "\": "; 
                    nodeAsString += serializeImpl(pair.second);
                    sep = ", ";
                }

                return nodeAsString + "}";
            }
    };
}

std::string utility::configuration::json::serialize(const ConfigurationNode& root) {
    SerializerImpl serializerImpl;
    return serializerImpl.serialize(root);
}
