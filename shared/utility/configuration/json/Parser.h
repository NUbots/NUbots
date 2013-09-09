#ifndef UTILITY_CONFIGURATION_CONFIGSYSTEM_JSON_PARSER_H
#define UTILITY_CONFIGURATION_CONFIGSYSTEM_JSON_PARSER_H
#include "../ConfigurationNode.h"
namespace utility {
namespace configuration {
namespace json {
    ConfigurationNode parse(const std::string& input);
}
}
}
#endif
