/*
 * This file is part of Darwin Hardware IO.
 *
 * Darwin Hardware IO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Darwin Hardware IO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Darwin Hardware IO.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MESSAGES_PLATFORM_DARWIN_DARWINSENSORS_H
#define MESSAGES_PLATFORM_DARWIN_DARWINSENSORS_H

#include <cstdint>
#include <string>
#include <stdexcept>

namespace messages {
    namespace platform {
        namespace darwin {

            struct LMissile {};

            struct RMissile {};

            /**
             * TODO document
             *
             * @author Trent Houliston
             */
            struct DarwinSensors {

                enum Error {
                    OK              = 0x0000,
                    INPUT_VOLTAGE   = 0x0001,
                    ANGLE_LIMIT     = 0x0002,
                    OVERHEATING     = 0x0004,
                    RANGE           = 0x0008,
                    CHECKSUM        = 0x0010,
                    OVERLOAD        = 0x0020,
                    INSTRUCTION     = 0x0040,
                    CORRUPT_DATA    = 0x0080,
                    TIMEOUT         = 0x0100
                };

                uint8_t cm730ErrorFlags;

                struct LEDPanel {
                    bool led2;
                    bool led3;
                    bool led4;
                } ledPanel;

                struct ColouredLED {
                    uint8_t r;
                    uint8_t g;
                    uint8_t b;
                };

                struct HeadLED : public ColouredLED {
                } headLED;

                struct EyeLED : public ColouredLED {
                } eyeLED;

                struct Buttons {
                    bool left;
                    bool middle;
                } buttons;

                float voltage;

                struct Acceleronometer {
                    float x;
                    float y;
                    float z;
                } acceleronometer;

                struct Gyroscope {
                    float x;
                    float y;
                    float z;
                } gyroscope;

                struct FSR {
                    float fsr1;
                    float fsr2;
                    float fsr3;
                    float fsr4;

                    float centreX;
                    float centreY;

                    uint8_t errorFlags;
                };

                struct FSRs {
                    FSR left;
                    FSR right;
                } fsr;

                struct Servo {

                    uint8_t errorFlags;

                    bool torqueEnabled;
                    bool led;

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
                    uint8_t temperature;
                };

                struct Servos {
                    Servo rShoulderPitch;
                    Servo lShoulderPitch;
                    Servo rShoulderRoll;
                    Servo lShoulderRoll;
                    Servo rElbow;
                    Servo lElbow;
                    Servo rHipYaw;
                    Servo lHipYaw;
                    Servo rHipRoll;
                    Servo lHipRoll;
                    Servo rHipPitch;
                    Servo lHipPitch;
                    Servo rKnee;
                    Servo lKnee;
                    Servo rAnklePitch;
                    Servo lAnklePitch;
                    Servo rAnkleRoll;
                    Servo lAnkleRoll;
                    Servo headPan;
                    Servo headTilt;

                    const Servo& operator[](int index) const;
                    Servo& operator[](int index);
                } servo;
            };
            
        }  // darwin
    }  // platform
}  // messages

#endif  // MESSAGES_PLATFORM_DARWIN_DARWINSENSORS_H
