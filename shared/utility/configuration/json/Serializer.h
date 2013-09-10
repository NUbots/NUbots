#ifndef UTILITY_CONFIGURATION_JSON_SERIALIZER_H
#define UTILITY_CONFIGURATION_JSON_SERIALIZER_H
#include "../ConfigurationNode.h"
namespace utility {
namespace configuration {
namespace json {
    std::string serialize(const ConfigurationNode& node); 
}
}
}
#endif
