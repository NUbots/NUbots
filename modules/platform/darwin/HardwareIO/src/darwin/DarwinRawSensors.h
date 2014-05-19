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

#ifndef DARWIN_DARWINRAWSENSORS_H
#define DARWIN_DARWINRAWSENSORS_H

#include <stdint.h>

namespace Darwin {

    /**
     * This namespace contains all of the raw Datatypes that we will use for reading and writing from the hardware
     */
    #pragma pack(push, 1) // Here we disable the OS putting in padding bytes so we can raw memcpy into this data
    namespace Types {

        /**
         * @brief This represents the raw gyro values from the CM730 (they are read from the board in ZYX)
         */
        struct Gyro {
            uint16_t z;
            uint16_t y;
            uint16_t x;
        };

        /**
         * @brief This represents the raw accelerometer values from the CM730
         */
        struct Accelerometer {
            uint16_t x;
            uint16_t y;
            uint16_t z;
        };

        /**
         * @brief This represents the data that comes from one of the MX28 motors
         */
        struct MX28Data {
            uint16_t presentPosition;
            uint16_t presentSpeed;
            uint16_t load;
            uint8_t voltage;
            uint8_t temperature;
        };

        /**
         * This represents data that comes from the Force Sensitive Resistors
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
         * @brief This represents data that comes from the CM730
         */
        struct CM730Data {
            uint8_t buttons;
            uint8_t reserved[7];
            Gyro gyroscope;
            Accelerometer accelerometer;
            uint8_t voltage;
        };

        /**
         * This is a type that is used control the motors, It is sent to the motors to cause a change
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
     * @brief This represents the bulk read results we get when we do one
     */
    struct BulkReadResults {
        /// @brief Holds data from the CM730
        Types::CM730Data cm730;
        /// @brief Holds data from the 20 servos (stored in location ServoID - 1)
        Types::MX28Data servos[20];
        /// @brief Holds data from the 2 FSR (stored as Right Left)
        Types::FSRData fsr[2];

        /// @brief Holds the error code (if any) from the CM730
        uint8_t cm730ErrorCode = 0;
        /// @brief Holds the error code from the Servos (ServoID - 1)
        uint8_t servoErrorCodes[20] = { 0 };
        /// @brief Holds the error code from the FSR (right then left)
        uint8_t fsrErrorCodes[2] = { 0 };
    };
    #pragma pack(pop) // Stop bitpacking our results
}  // namespace Darwin

#endif
