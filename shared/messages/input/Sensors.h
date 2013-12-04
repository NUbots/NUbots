/*
 * This file is part of AudioInput.
 *
 * AudioInput is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * AudioInput is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with AudioInput.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MESSAGES_INPUT_SENSORS_H
#define MESSAGES_INPUT_SENSORS_H

#include "ServoID.h"

namespace messages {
    namespace input {
        
        struct Sensors {
        
            struct Servo {
                uint8_t errorFlags;

                ServoID id;

                bool enabled;

                float pGain;
                float iGain;
                float dGain;

                float goalPosition;
                float movingSpeed;
                float torqueLimit;

                float presentPosition;
                float presentSpeed;

                float load;
                float voltage;
                float temperature;
            };

            struct Accelerometer {
                float x;
                float y;
                float z;
            };
            
            struct Gyroscope {
                float x;
                float y;
                float z;
            };

            struct FSR {
                float fsr1;
                float fsr2;
                float fsr3;
                float fsr4;

                float centreX;
                float centreY;
            };
        };
        
        Acceleronometer acceleronometer;
        Gyroscope gyroscope;
        FSR leftFSR;
        FSR rightFSR;
        Servo servos[20];
    }
}

#endif