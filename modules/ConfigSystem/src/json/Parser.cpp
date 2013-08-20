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

    bool Parser::readif(std::string::iterator current, char target) {
        if(*current == target) {
            ++current;
            return true;
        }

        return false;
    }

    message::ConfigurationNode Parser::parseJson(const std::string& input) {
        begin = std::begin(input);
        end = std::end(input);

        if(peek(begin) == '{') {
            return parseObject(begin, end);
        } else if(peek(begin) == '[') {
            return parseArray(begin, end);
        } else {
            throw new std::runtime_exception("Expected { or [ as starting token but found: " + *begin);
        }
    }

    message::ConfigurationNode Parser::parseObject(std::string::iterator current, std::string::iterator end) {
        if(readif(current, '{')) {
            if(readif(current, '}')) {
                return message::ConfigurationNode;
            }
        }

        message::ConfigurationNode members(parseMembers(current, end));

        if(readif(current, '}')) {
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
