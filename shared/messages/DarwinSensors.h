/*
 * This file is part of DarwinPlatform.
 *
 * DarwinPlatform is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * DarwinPlatform is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with DarwinPlatform.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#ifndef MESSAGES_DARWINSENSORS_H
#define MESSAGES_DARWINSENSORS_H

#include <stdint.h>
#include <stdexcept>

namespace Messages {

    /**
     * TODO
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
            enum ID {
                R_SHOULDER_PITCH    = 0,
                L_SHOULDER_PITCH    = 1,
                R_SHOULDER_ROLL     = 2,
                L_SHOULDER_ROLL     = 3,
                R_ELBOW             = 4,
                L_ELBOW             = 5,
                R_HIP_YAW           = 6,
                L_HIP_YAW           = 7,
                R_HIP_ROLL          = 8,
                L_HIP_ROLL          = 9,
                R_HIP_PITCH         = 10,
                L_HIP_PITCH         = 11,
                R_KNEE              = 12,
                L_KNEE              = 13,
                R_ANKLE_PITCH       = 14,
                L_ANKLE_PITCH       = 15,
                R_ANKLE_ROLL        = 16,
                L_ANKLE_ROLL        = 17,
                HEAD_PAN            = 18,
                HEAD_TILT           = 19
            };

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
};

#endif