#ifndef MESSAGES_DARWINSERVOS_H
#define MESSAGES_DARWINSERVOS_H

#include <vector>

namespace Messages {

    /**
     * TODO
     */
    struct DarwinServoCommand {
        uint8_t servoId;
        float pGain;
        float iGain;
        float dGain;
        float goalPosition;
        float movingSpeed;
    };

    /**
     * TODO this is the commands for the sensors in "Standard Space"
     */
    struct DarwinServoCommands {
        std::vector<DarwinServoCommand> commands;
    };
};

#endif