#ifndef MODULES_CONFIGSYSTEM_JSON_H
#define MODULES_CONFIGSYSTEM_JSON_H
#include "messages/ConfigurationNode.h"
namespace modules {
namespace ConfigSystem {
namespace json {
    public class Parser {
        public:
            message::ConfigurationNode parseJson(const std::string& input);
        private:
            std::string::iterator begin;
            std::string::iterator end;

            char peek(std::string::iterator current);
            char read(std::string::iterator current);
            bool readif(std::string::iterator current, char target);

            message::ConfigurationNode parseObject(std::string::iterator current, std::string::iterator end);
            message::ConfigurationNode parseMembers(std::string::iterator current, std::string::iterator end);
            message::ConfigurationNode parsePair(std::string::iterator current, std::string::iterator end);
            message::ConfigurationNode parseArray(std::string::iterator current, std::string::iterator end);
            message::ConfigurationNode parseElements(std::string::iterator current, std::string::iterator end);
            message::ConfigurationNode parseValue(std::string::iterator current, std::string::iterator end);
    }
}
}
}
#endif
