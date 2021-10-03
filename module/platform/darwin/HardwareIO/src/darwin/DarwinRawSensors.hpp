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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef DARWIN_DARWINRAWSENSORS_HPP
#define DARWIN_DARWINRAWSENSORS_HPP

#include <cstdint>

namespace Darwin {

/**
 * This namespace contains all of the raw Datatypes that we will use for reading and writing from the hardware
 */
#pragma pack(push, 1)  // Here we disable the OS putting in padding bytes so we can raw memcpy into this data
    namespace Types {

        /**
         * @brief This represents the raw gyro values from the CM740 (they are read from the board in ZYX)
         */
        struct Gyro {
            Gyro() = default;
            uint16_t z{0};
            uint16_t y{0};
            uint16_t x{0};
        };

        /**
         * @brief This represents the raw accelerometer values from the CM740
         */
        struct Accelerometer {
            Accelerometer() = default;
            uint16_t x{0};
            uint16_t y{0};
            uint16_t z{0};
        };

        /**
         * @brief This represents the data that comes from one of the MX28 motors
         */
        struct MX28Data {
            MX28Data() = default;
            uint16_t presentPosition{0};
            uint16_t presentSpeed{0};
            uint16_t load{0};
            uint8_t voltage{0};
            uint8_t temperature{0};
        };

        /**
         * This represents data that comes from the Force Sensitive Resistors
         */
        struct FSRData {
            FSRData() = default;
            uint16_t fsr1{0};
            uint16_t fsr2{0};
            uint16_t fsr3{0};
            uint16_t fsr4{0};
            uint8_t centreX{0};
            uint8_t centreY{0};
        };

        /**
         * @brief This represents data that comes from the CM740
         */
        struct CM740Data {
            CM740Data() = default;
            uint8_t buttons{0};
            uint8_t reserved[7] = {0};
            Gyro gyroscope;
            Accelerometer accelerometer;
            uint8_t voltage{0};
        };

        /**
         * This is a type that is used control the motors, It is sent to the motors to cause a change
         */
        struct ServoValues {
            ServoValues() = default;
            uint8_t servoId{0};
            uint8_t dGain{0};
            uint8_t iGain{0};
            uint8_t pGain{0};
            uint8_t reserved{0};
            uint16_t goalPostion{0};
            uint16_t movingSpeed{0};
        };
    }  // namespace Types

    /**
     * @brief This represents the bulk read results we get when we do one
     */
    struct BulkReadResults {
        BulkReadResults() = default;
        /// @brief Holds data from the CM740
        Types::CM740Data cm740;
        /// @brief Holds data from the 20 servos (stored in location ServoID - 1)
        Types::MX28Data servos[20] = {Types::MX28Data()};
        /// @brief Holds data from the 2 FSR (stored as Right Left)
        Types::FSRData fsr[2] = {Types::FSRData()};

        /// @brief Holds the error code (if any) from the CM740
        uint8_t cm740ErrorCode = 0;
        /// @brief Holds the error code from the Servos (ServoID - 1)
        uint8_t servoErrorCodes[20] = {0};
        /// @brief Holds the error code from the FSR (right then left)
        uint8_t fsrErrorCodes[2] = {0};
    };
#pragma pack(pop)  // Stop bitpacking our results
}  // namespace Darwin

#endif
