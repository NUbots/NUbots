#ifndef MODULES_CONFIGSYSTEM_JSON_SERIALIZER_H
#define MODULES_CONFIGSYSTEM_JSON_SERIALIZER_H
#include "messages/ConfigurationNode.h"
namespace modules {
namespace json {
    class Serializer {
        public:
            static std::string serialize(const messages::ConfigurationNode& node); 
    };
}
}
#endif
