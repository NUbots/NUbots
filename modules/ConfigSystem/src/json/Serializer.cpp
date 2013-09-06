#include "Serializer.h"

#include <vector>
#include <cctype>
#include <sstream>
using namespace modules;
using namespace json;

namespace {
    class SerializerImpl {
        public:
            std::string serialize(const messages::ConfigurationNode& root) {
                return serializeImpl(root);
            }
        private:
            std::string serializeImpl(const messages::ConfigurationNode& node) {
                using messages;

                switch(node.nativeType()) {
                    case ConfigurationNode::STRING: 
                    case ConfigurationNode::INTEGER:
                    case ConfigurationNode::FLOATINGPOINT:
                    case ConfigurationNode::BOOLEAN:
                    case ConfigurationNode::NULLPOINTER:
                        return serializeNullpointer(node);
                        break;
                    case ConfigurationNode::ARRAY:
                        return serializeArray(node);
                        break;
                    case ConfigurationNode::OBJECT:
                        return serializeObject(node);
                        break;
                }

                // TODO: Add extra info here
                throw std::runtime_error(std::string("Bad node found: "));
            }

            std::string serializePrimative(const messages::ConfigurationNode& node) {
                std::string nodeAsString = node;

                if(node.nativeType() == ConfigurationNode::STRING) {
                    nodeAsString = "\"" + nodeAsString + "\"";
                }

                return nodeAsString;
            }

            std::string serializeArray(const messages::ConfigurationNode& node) {
                std::string nodeAsString = "[";
                std::vector<ConfigurationNode> children = node;
            }
    }

    class ParseImpl {
        public:
            ParseImpl(std::string input, std::vector<jsmntok_t> tokens) :
                input(input),
                tokens(tokens),
                curTok(0) {
            }

            messages::ConfigurationNode parse() {
                return parseImpl(read());
            }
        private:
            messages::ConfigurationNode parseImpl(jsmntok_t root) {
                switch(root.type) {
                    case JSMN_OBJECT:
                        return parseObject(root);
                        break;
                    case JSMN_ARRAY:
                        return parseArray(root);
                        break;
                    case JSMN_STRING:
                        return parseString(root);
                        break;
                    case JSMN_PRIMITIVE:
                        return parsePrimative(root);
                        break;
                }

                throw std::runtime_error(std::string("Bad node found: ") + getData(root));
            }

            messages::ConfigurationNode parseObject(jsmntok_t token) {
                messages::ConfigurationNode object;
                for(int i = 0; i < token.size; ++++i) {
                    auto key = getData(read());
                    auto valnode = parseImpl(read());

                    object.add(key, valnode);
                }
                return std::move(object);
            }

            messages::ConfigurationNode parseArray(jsmntok_t token) {
                std::vector<messages::ConfigurationNode> array;
                for(int i = 0; i < token.size; ++i) {
                    array.push_back(parseImpl(read()));
                }
                return messages::ConfigurationNode(array);
            }

            messages::ConfigurationNode parseString(jsmntok_t token) {
                std::string value = getData(token);

                return std::move(messages::ConfigurationNode(value));
            }

            messages::ConfigurationNode parsePrimative(jsmntok_t token) {
                // First we need to figure out the data type of the primative based
                // on it's value.
                std::string value = getData(token);

                // We can determine the type using the first character
                // see: https://bitbucket.org/zserge/jsmn/src
                char first = value[0];
                if(first == 't') {
                    return messages::ConfigurationNode(true);
                } else if(first == 'f') {
                    return messages::ConfigurationNode(false);
                } else if(first == 'n') {
                    return messages::ConfigurationNode();
                } else if(first == '-' || isdigit(first)) {
                    // If the string contains a dot then it's a float
                    if(value.find('.') != std::string::npos) {
                        std::stringstream stream(value);
                        float valueAsFloat;
                        stream >> valueAsFloat;

                        return messages::ConfigurationNode(valueAsFloat);
                    } else {
                        std::stringstream stream(value);
                        int valueAsInt;
                        stream >> valueAsInt;

                        return messages::ConfigurationNode(valueAsInt);
                    }
                }

                throw std::runtime_error(std::string("Bad primative: ") + value);
            }

            jsmntok_t peek() {
                return tokens[curTok + 1];
            }

            jsmntok_t read() {
                return tokens[curTok++];
            }

            std::string getData(jsmntok_t token) {
                return input.substr(token.start, token.end - token.start);
            }

            std::string input;
            std::vector<jsmntok_t> tokens;
            int curTok;
    };
}

std::string Serializer::serialize(const messages::ConfigurationNode& root) {

}

messages::ConfigurationNode Parser::parse(std::string input) {
    jsmn_parser parser;
    jsmn_init(&parser);

    int tokenBufferSize = input.size();

    std::vector<jsmntok_t> tokens;
    tokens.resize(tokenBufferSize);

    while(true) {
        jsmnerr_t result = jsmn_parse(&parser, input.c_str(), tokens.data(), tokenBufferSize);

        if(result == JSMN_SUCCESS) {
            break;
        } else if(result == JSMN_ERROR_NOMEM) {
            tokenBufferSize *= 2;
            tokens.resize(tokenBufferSize);
        }
    }

    // From this point down the size parameter of tokens is unreliazble. Effectively
    // we have an array-backed tree which we need to recurse.
    ParseImpl parseImpl(input, tokens);
    return parseImpl.parse();
}
