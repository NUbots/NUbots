#ifndef MODULES_CONFIGSYSTEM_JSON_PARSER_H
#define MODULES_CONFIGSYSTEM_JSON_PARSER_H
#include "messages/ConfigurationNode.h"
namespace modules {
namespace json {
    class Parser {
        public:
            Messages::ConfigurationNode parse(std::string input); 
        private:
            Messages::ConfigurationNode parseImpl();
    };
}
}
#endif
