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

#ifndef DARWIN_CM730_H_
#define DARWIN_CM730_H_

#include "DarwinDevice.h"

#ifdef VERSION
#undef VERSION
#endif

namespace Darwin {

/**
 * @brief This represents, and gives access to the darwin's CM730 subboard
 *
 * @author Trent Houliston
 */
class CM730 : public DarwinDevice {
public:
    /**
     * @brief Holds the addresses of the various bytes in the CM730.
     *
     * @details
     *  for additional details see
     * http://support.robotis.com/en/product/controller/opencr.htm
     */
    enum Address {
        MODEL_NUMBER_L    = 0,
        MODEL_NUMBER_H    = 1,
        VERSION           = 2,
        ID                = 3,
        BAUD_RATE         = 4,
        RETURN_DELAY_TIME = 5,
        RETURN_LEVEL      = 16,
        DXL_POWER         = 24,
        LED_PANNEL        = 25,
        LED_RGB_L         = 26,
        LED_RGB_H         = 27,
        BUZZER_L          = 28,
        BUZZER_H          = 29,
        BUTTON            = 30,
        VOLTAGE           = 31,
        GYRO_Z_L          = 32,
        GYRO_Z_H          = 33,
        GYRO_Y_L          = 34,
        GYRO_Y_H          = 35,
        GYRO_X_L          = 36,
        GYRO_X_H          = 37,
        ACCEL_X_L         = 38,
        ACCEL_X_H         = 39,
        ACCEL_Y_L         = 40,
        ACCEL_Y_H         = 41,
        ACCEL_Z_L         = 42,
        ACCEL_Z_H         = 43,
        ROLL_L            = 44,
        ROLL_H            = 45,
        PITCH_L           = 46,
        PITCH_H           = 47,
        YAW_L             = 48,
        YAW_H             = 49,
        IMU_CONTROL       = 50
    };

    CM730(UART& coms, int id);

    /**
     * @brief turns on the Dynamixel power (servos and foot sensors)
     */
    void turnOnDynamixel();
};
}  // namespace Darwin

#endif
