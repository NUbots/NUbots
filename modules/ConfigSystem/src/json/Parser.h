#ifndef MODULES_CONFIGSYSTEM_JSON_PARSER_H
#define MODULES_CONFIGSYSTEM_JSON_PARSER_H
#include "messages/ConfigurationNode.h"
namespace modules {
namespace json {
    class Parser {
        public:
            static messages::ConfigurationNode parse(const std::string& input); 
    };
}
}
#endif
