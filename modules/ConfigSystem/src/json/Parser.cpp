#include "Parser.h"
#include "jsmn.h"

#include <vector>
#include <iostream>
#include <cctype>
#include <sstream>
using namespace modules;
using namespace json;

namespace {
    class ParseImpl {
        public:
            ParseImpl(std::string input ,std::vector<jsmntok_t> tokens) :
                input(input),
                tokens(tokens),
                curTok(0) {
            }

            Messages::ConfigurationNode parse() {
                return parseImpl(read());
            }
        private:
            Messages::ConfigurationNode parseImpl(jsmntok_t root) {
                std::cout << "parseImpl " << getData(root) << " Type: " << root.type << std::endl;
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
                std::cout << "/parseImpl " << std::endl;

                throw std::runtime_error(std::string("Bad node found: ") + getData(root));
            }

            Messages::ConfigurationNode parseObject(jsmntok_t token) {
                std::cout << "parseObject " << getData(token) << std::endl;
                Messages::ConfigurationNode object;
                for(int i = 0; i < token.size; ++++i) {
                    auto key = getData(read());
                    auto valnode = parseImpl(read());

                    object.add(key, valnode);
                }
                std::cout << "/parseObject";
                return std::move(object);
            }

            Messages::ConfigurationNode parseArray(jsmntok_t token) {
                std::cout << "parseArray " << getData(token) << std::endl;
                std::vector<Messages::ConfigurationNode> array;
                for(int i = 0; i < token.size; ++i) {
                    array.push_back(parseImpl(read()));
                }
                std::cout << "/parseArray" << std::endl;
                return Messages::ConfigurationNode(array);
            }

            Messages::ConfigurationNode parseString(jsmntok_t token) {
                std::string value = getData(token);

                return std::move(Messages::ConfigurationNode(value));
            }

            Messages::ConfigurationNode parsePrimative(jsmntok_t token) {
                // First we need to figure out the data type of the primative based
                // on it's value.
                std::string value = getData(token);

                // We can determine the type using the first character
                // see: https://bitbucket.org/zserge/jsmn/src
                char first = value[0];
                if(first == 't') {
                    return Messages::ConfigurationNode(true);
                } else if(first == 'f') {
                    return Messages::ConfigurationNode(false);
                } else if(first == 'n') {
                    return Messages::ConfigurationNode();
                } else if(first == '-' || isdigit(first)) {
                    // If the string contains a dot then it's a float
                    if(value.find('.') != std::string::npos) {
                        std::stringstream stream(value);
                        float valueAsFloat;
                        stream >> valueAsFloat;

                        return Messages::ConfigurationNode(valueAsFloat);
                    } else {
                        std::stringstream stream(value);
                        int valueAsInt;
                        stream >> valueAsInt;

                        return Messages::ConfigurationNode(valueAsInt);
                    }
                }

                throw std::runtime_error(std::string("Bad primative: ") + value);
            }

            jsmntok_t peek() {
                return tokens[curTok + 1];
            }

            jsmntok_t read() {
                std::cout << "Reading: " << getData(tokens[curTok]) << std::endl;
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

Messages::ConfigurationNode Parser::parse(std::string input) {
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

    // Parse each of the tokens into our configuration node.
    /*int numTokens = tokens[0].size;
    std::cout << "NumTokens: " << numTokens << std::endl;
    for(unsigned int i = 0; i <= tokens.size(); ++i) {
        jsmntok_t token = tokens[i];
        std::string val(input, token.start, token.end - token.start);
        std::cout << "Token " << i << ": " << val << std::endl;
    }

    return Messages::ConfigurationNode{};*/
}
