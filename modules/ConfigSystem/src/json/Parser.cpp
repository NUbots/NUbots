#include "Parser.h"
namespace modules {
namespace ConfigSystem {
namespace json {

    char Parser::peek(std::string::iterator current) {
        return *current;
    }

    char Parser::read(std::string::iterator current) {
        return *(current++);
    }

    message::ConfigurationNode Parser::parseJson(const std::string& input) {
        auto begin = std::begin(input);
        auto end = std::end(input);

        if(*begin == '{') {
            return parseObject(begin, end);
        } else if(*begin == '[') {
            return parseArray(begin, end);
        } else {
            throw new std::runtime_exception("Expected { or [ as starting token but found: " + *begin);
        }
    }

    message::ConfigurationNode Parser::parseObject(std::string::iterator current, std::string::iterator end) {
        if(*current == '{') {
            ++current;
            if(*current == '}') {
                return message::ConfigurationNode;
            }
        }

        message::ConfigurationNode members(parseMembers(current, end));

        if(*current == '}') {
            ++current;
            return members;
        } else {
            throw new std::runtime_exception("Expected } in object but found " + *current);
        }
    }

    message::ConfigurationNode Parser::parseMembers(std::string::iterator current, std::string::iterator end) {
        message::ConfigurationNode members;

        do {
            members.add(parsePair(current, end));
        } while(peek(current) == ',' && read(current))

        return members;
    }

    message::ConfigurationNode Parser::parsePair(std::string::iterator current, std::string::iterator end) {
        
    }

    message::ConfigurationNode Parser::parseArray(std::string::iterator current, std::string::iterator end);
    message::ConfigurationNode Parser::parseElements(std::string::iterator current, std::string::iterator end);
    message::ConfigurationNode Parser::parseValue(std::string::iterator current, std::string::iterator end);
}
}
}
