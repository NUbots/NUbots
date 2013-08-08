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

#ifndef DARWIN_DARWINRAWSENSORS_H
#define DARWIN_DARWINRAWSENSORS_H

#include <stdint.h>

namespace Darwin {

    /**
     * TODO
     */
    #pragma pack(push, 1) // Here we disable the OS putting in padding bytes so we can raw memcpy into this data
    namespace Types {

        /**
         * TODO
         */
        struct Gyro {
            uint16_t z;
            uint16_t y;
            uint16_t x;
        };

        /**
         * TODO
         */
        struct Accelerometer {
            uint16_t x;
            uint16_t y;
            uint16_t z;
        };

        /**
         * TODO
         */
        struct MX28Data {
            bool torqueEnabled;
            bool LED;
            uint8_t dGain;
            uint8_t iGain;
            uint8_t pGain;
            uint8_t reservedByte;
            uint16_t goalPosition;
            uint16_t movingSpeed;
            uint16_t torqueLimit;
            uint16_t presentPosition;
            uint16_t presentSpeed;
            uint16_t load;
            uint8_t voltage;
            uint8_t temperature;
        };

        /**
         * TODO
         */
        struct FSRData {
            uint16_t fsr1;
            uint16_t fsr2;
            uint16_t fsr3;
            uint16_t fsr4;
            uint8_t centreX;
            uint8_t centreY;
        };

        /**
         * TODO
         */
        struct CM730Data {
            uint8_t ledPanel;
            uint16_t headLED;
            uint16_t eyeLED;
            uint8_t buttons;
            uint8_t reserved[7];
            Gyro gyroscope;
            Accelerometer acceleronometer;
            uint8_t voltage;
        };

        /**
         * TODO
         */
        struct ServoValues {
            uint8_t servoId;
            uint8_t dGain;
            uint8_t iGain;
            uint8_t pGain;
            uint8_t reserved;
            uint16_t goalPostion;
            uint16_t movingSpeed;
        };
    }  // namespace Types

    /**
     * TODO
     */
    struct BulkReadResults {
        Types::CM730Data cm730;
        Types::MX28Data servos[20];
        Types::FSRData fsr[2];

        uint8_t cm730ErrorCode = 0;
        uint8_t servoErrorCodes[20] = { 0 };
        uint8_t fsrErrorCodes[2] = { 0 };
    };
    #pragma pack(pop)
}  // namespace Darwin

#endif
