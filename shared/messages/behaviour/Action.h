#ifndef MESSAGES_BEHAVIOUR_ACTIONS_H
#define MESSAGES_BEHAVIOUR_ACTIONS_H

#include <nuclear>
#include "messages/input/ServoID.h"

namespace messages {
    namespace behaviour {
        
        enum class LimbID {
            LEFT_LEG = 0,
            RIGHT_LEG = 1,
            LEFT_ARM = 2,
            RIGHT_ARM = 3,
            HEAD = 4
        };
        
        struct RegisterAction {
            
            size_t id;
            
            std::vector<std::pair<float, std::set<LimbID>>> limbSet;
            
            std::function<void (std::set<LimbID>)> start;
            std::function<void (std::set<LimbID>)> kill;
        };
        
        struct ActionPriorites {
            size_t id;
            
            std::vector<float> priorities;
        };

        struct ServoCommand {
            size_t source;
            
            NUClear::clock::time_point time;
            input::ServoID id;
            float position;
            float gain;
        };
    }
}

#endif

