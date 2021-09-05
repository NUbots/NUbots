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

#ifndef DARWIN_MX28_HPP
#define DARWIN_MX28_HPP

#include "DarwinDevice.hpp"

namespace Darwin {
    /**
     * @brief This represents, and gives access to the darwin's MX28 motors
     *
     * @author Trent Houliston
     */
    class MX28 : public DarwinDevice {

    public:
        /**
         * @brief This enum holds the addresses of the various bytes in the MX28.
         *
         * @details
         *  for additional details see http://support.robotis.com/en/product/dynamixel/mx_series/mx-28.htm
         */
        enum Address {
            MODEL_NUMBER_L         = 0,
            MODEL_NUMBER_H         = 1,
            VERSION                = 2,
            ID                     = 3,
            BAUD_RATE              = 4,
            RETURN_DELAY_TIME      = 5,
            CW_ANGLE_LIMIT_L       = 6,
            CW_ANGLE_LIMIT_H       = 7,
            CCW_ANGLE_LIMIT_L      = 8,
            CCW_ANGLE_LIMIT_H      = 9,
            SYSTEM_DATA2           = 10,
            HIGH_LIMIT_TEMPERATURE = 11,
            LOW_LIMIT_VOLTAGE      = 12,
            HIGH_LIMIT_VOLTAGE     = 13,
            MAX_TORQUE_L           = 14,
            MAX_TORQUE_H           = 15,
            RETURN_LEVEL           = 16,
            ALARM_LED              = 17,
            ALARM_SHUTDOWN         = 18,
            OPERATING_MODE         = 19,
            LOW_CALIBRATION_L      = 20,
            LOW_CALIBRATION_H      = 21,
            HIGH_CALIBRATION_L     = 22,
            HIGH_CALIBRATION_H     = 23,
            TORQUE_ENABLE          = 24,
            LED                    = 25,
            D_GAIN                 = 26,
            I_GAIN                 = 27,
            P_GAIN                 = 28,
            RESERVED               = 29,
            GOAL_POSITION_L        = 30,
            GOAL_POSITION_H        = 31,
            MOVING_SPEED_L         = 32,
            MOVING_SPEED_H         = 33,
            TORQUE_LIMIT_L         = 34,
            TORQUE_LIMIT_H         = 35,
            PRESENT_POSITION_L     = 36,
            PRESENT_POSITION_H     = 37,
            PRESENT_SPEED_L        = 38,
            PRESENT_SPEED_H        = 39,
            PRESENT_LOAD_L         = 40,
            PRESENT_LOAD_H         = 41,
            PRESENT_VOLTAGE        = 42,
            PRESENT_TEMPERATURE    = 43,
            REGISTERED_INSTRUCTION = 44,
            PAUSE_TIME             = 45,
            MOVING                 = 46,
            LOCK                   = 47,
            PUNCH_L                = 48,
            PUNCH_H                = 49,
            RESERVED4              = 50,
            RESERVED5              = 51,
            POT_L                  = 52,
            POT_H                  = 53,
            PWM_OUT_L              = 54,
            PWM_OUT_H              = 55,
            P_ERROR_L              = 56,
            P_ERROR_H              = 57,
            I_ERROR_L              = 58,
            I_ERROR_H              = 59,
            D_ERROR_L              = 60,
            D_ERROR_H              = 61,
            P_ERROR_OUT_L          = 62,
            P_ERROR_OUT_H          = 63,
            I_ERROR_OUT_L          = 64,
            I_ERROR_OUT_H          = 65,
            D_ERROR_OUT_L          = 66,
            D_ERROR_OUT_H          = 67
        };

        MX28(UART& coms, int id);
    };
}  // namespace Darwin

#endif
