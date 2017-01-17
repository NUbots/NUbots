/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MESSAGE_PLATFORM_DARWIN_DARWINSENSORS_H
#define MESSAGE_PLATFORM_DARWIN_DARWINSENSORS_H

#include <nuclear>
#include <cstdint>
#include <string>
#include <stdexcept>

#include "message/input/proto/Sensors.h"

namespace message {
namespace platform {
namespace darwin {

    /**
     * TODO document
     *
     * @author Trent Houliston
     */
    struct DarwinSensors {
        DarwinSensors()
            : timestamp()
            , cm730ErrorFlags(0)
            , ledPanel()
            , headLED()
            , eyeLED()
            , buttons()
            , voltage(0.0f)
            , accelerometer()
            , gyroscope()
            , fsr()
            , servo()
            {}

        // bitmask values
        enum Error {
            OK              = 0, // not really a flag but the lack of any other flag
            INPUT_VOLTAGE   = 1 << 0,
            ANGLE_LIMIT     = 1 << 1,
            OVERHEATING     = 1 << 2,
            RANGE           = 1 << 3,
            CHECKSUM        = 1 << 4,
            OVERLOAD        = 1 << 5,
            INSTRUCTION     = 1 << 6,
            CORRUPT_DATA    = 1 << 7,
            TIMEOUT         = 1 << 8,
            TIMEOUT_VICTIM  = 1 << 9
        };

        NUClear::clock::time_point timestamp;

        uint16_t cm730ErrorFlags;

        struct LEDPanel {
            LEDPanel() : led2(false), led3(false), led4(false) {}
            LEDPanel(bool LED2, bool LED3, bool LED4) : led2(LED2), led3(LED3), led4(LED4) {}
            bool led2;
            bool led3;
            bool led4;
        } ledPanel;

        struct HeadLED {
            HeadLED() : r(0), g(0), b(0) {}
            HeadLED(uint8_t red, uint8_t green, uint8_t blue) : r(red), g(green), b(blue) {}
            uint8_t r;
            uint8_t g;
            uint8_t b;
        } headLED;

        struct EyeLED {
            EyeLED() : r(0), g(0), b(0) {}
            EyeLED(uint8_t red, uint8_t green, uint8_t blue) : r(red), g(green), b(blue) {}
            uint8_t r;
            uint8_t g;
            uint8_t b;
        } eyeLED;

        struct Buttons {
            Buttons() : left(false), middle(false) {}
            Buttons(bool l, bool m) : left(l), middle(m) {}
            bool left;
            bool middle;
        } buttons;

        float voltage;

        struct Accelerometer {
            Accelerometer() : x(0.0f), y(0.0f), z(0.0f) {}
            Accelerometer(float x, float y, float z) : x(x), y(y), z(z) {}
            float x;
            float y;
            float z;
        } accelerometer;

        struct Gyroscope {
            Gyroscope() : x(0.0f), y(0.0f), z(0.0f) {}
            Gyroscope(float x, float y, float z) : x(x), y(y), z(z) {}
            float x;
            float y;
            float z;
        } gyroscope;

        struct FSR {
            FSR() : fsr1(0.0f), fsr2(0.0f), fsr3(0.0f), fsr4(0.0f), centreX(0.0f), centreY(0.0f), errorFlags(0) {}
            FSR(float fsr1, float fsr2, float fsr3, float fsr4, float centreX, float centreY, uint16_t error)
                    : fsr1(fsr1), fsr2(fsr2), fsr3(fsr3), fsr4(fsr4), centreX(centreX), centreY(centreY), errorFlags(error) {}
            float fsr1;
            float fsr2;
            float fsr3;
            float fsr4;

            float centreX;
            float centreY;

            uint16_t errorFlags;
        };

        struct FSRs {
            FSRs() : left(), right() {}
            FSRs(const FSR& left, const FSR& right) : left(left), right(right) {}
            FSR left;
            FSR right;
        } fsr;

        struct Servo {
            Servo() : errorFlags(0), torqueEnabled(false), pGain(0.0f), iGain(0.0f), dGain(0.0f), goalPosition(0.0f),
                      movingSpeed(0.0f), torque(0.0f), presentPosition(0.0f), presentSpeed(0.0f), load(0.0f),
                      voltage(0.0f), temperature(0) {}
            Servo(const Servo& other)
                : errorFlags(other.errorFlags), torqueEnabled(other.torqueEnabled), pGain(other.pGain), iGain(other.iGain),
                dGain(other.dGain), goalPosition(other.goalPosition), movingSpeed(other.movingSpeed), torque(other.torque),
                presentPosition(other.presentPosition), presentSpeed(other.presentSpeed), load(other.load),
                voltage(other.voltage), temperature(other.temperature) {}

            uint16_t errorFlags;

            bool torqueEnabled;

            float pGain;
            float iGain;
            float dGain;

            float goalPosition;
            float movingSpeed;

            float torque;

            float presentPosition;
            float presentSpeed;

            float load;
            float voltage;
            uint8_t temperature;
        };

        struct Servos {
            Servos() : rShoulderPitch(), lShoulderPitch(), rShoulderRoll(), lShoulderRoll(), rElbow(), lElbow(),
                       rHipYaw(), lHipYaw(), rHipRoll(), lHipRoll(), rHipPitch(), lHipPitch(), rKnee(), lKnee(),
                       rAnklePitch(), lAnklePitch(), rAnkleRoll(), lAnkleRoll(), headPan(), headTilt() {}
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

            const Servo& operator[](message::input::proto::Sensors::ServoID::Value index) const;
            Servo& operator[](message::input::proto::Sensors::ServoID::Value index);

            const Servo& operator[](int index) const;
            Servo& operator[](int index);

        } servo;
    };

    // Button press events
    struct ButtonLeftDown {};
    struct ButtonLeftUp {};
    struct ButtonMiddleDown {};
    struct ButtonMiddleUp {};

}  // darwin
}  // platform
}  // message

#endif  // MESSAGE_PLATFORM_DARWIN_DARWINSENSORS_H
