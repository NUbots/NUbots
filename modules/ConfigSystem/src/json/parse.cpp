#include "parse.h"
#include "jsmn.h"

#include <vector>
#include <iostream>
using namespace modules;
using namespace json;

Messages::ConfigurationNode modules::json::parse(std::string input) {
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

    // Parse each of the tokens into our configuration node.
    int numTokens = tokens[0].size;
    std::cout << "NumTokens: " << numTokens << std::endl;
    for(unsigned int i = 0; i <= tokens.size(); ++i) {
        jsmntok_t token = tokens[i];
        std::string val(input, token.start, token.end - token.start);
        std::cout << "Token " << i << ": " << val << std::endl;
    }

    return Messages::ConfigurationNode{};
}
