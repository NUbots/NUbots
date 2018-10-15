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

#ifndef DARWIN_MX106_H
#define DARWIN_MX106_H

#include "DarwinDevice.h"

#define SIZE_MX64_MODEL_NUMBER 2
#define SIZE_MX64_MODEL_INFORMATION 4
#define SIZE_MX64_VERSION_OF_FIRMWARE 1
#define SIZE_MX64_ID 1
#define SIZE_MX64_BAUD_RATE 1
#define SIZE_MX64_RETURN_DELAY_TIME 1
#define SIZE_MX64_DRIVE_MODE 1
#define SIZE_MX64_OPERATING_MODE 1
#define SIZE_MX64_SECONDARY_SHADOW_ID 1
#define SIZE_MX64_PROTOCOL_VERSION 1
#define SIZE_MX64_HOMING_OFFSET 4
#define SIZE_MX64_MOVING_THRESHOLD 4
#define SIZE_MX64_TEMPERATURE_LIMIT 1
#define SIZE_MX64_MAX_VOLTAGE_LIMIT 2
#define SIZE_MX64_MIN_VOLTAGE_LIMIT 2
#define SIZE_MX64_PWM_LIMIT 2
#define SIZE_MX64_CURRENT_LIMIT 2
#define SIZE_MX64_ACCELERATION_LIMIT 4
#define SIZE_MX64_VELOCITY_LIMIT 4
#define SIZE_MX64_MAX_POSITION_LIMIT 4
#define SIZE_MX64_MIN_POSITION_LIMIT 4
#define SIZE_MX64_SHUTDOWN 1
#define SIZE_MX64_TORQUE_ENABLE 1
#define SIZE_MX64_LED 1
#define SIZE_MX64_STATUS_RETURN_LEVEL 1
#define SIZE_MX64_REGISTERED_INSTRUCTION 1
#define SIZE_MX64_HARDWARE_ERROR_STATUS 1
#define SIZE_MX64_VELOCITY_I_GAIN 2
#define SIZE_MX64_VELOCITY_P_GAIN 2
#define SIZE_MX64_POSITION_D_GAIN 2
#define SIZE_MX64_POSITION_I_GAIN 2
#define SIZE_MX64_POSITION_P_GAIN 2
#define SIZE_MX64_FEEDFORWARD_2ND_GAIN 2
#define SIZE_MX64_FEEDFORWARD_1ST_GAIN 2
#define SIZE_MX64_BUS_WATCHDOG 1
#define SIZE_MX64_GOAL_PWM 2
#define SIZE_MX64_GOAL_CURRENT 2
#define SIZE_MX64_GOAL_VELOCITY 4
#define SIZE_MX64_PROFILE_ACCELERATION 4
#define SIZE_MX64_PROFILE_VELOCITY 4
#define SIZE_MX64_GOAL_POSITION 4
#define SIZE_MX64_REALTIME_TICK 2
#define SIZE_MX64_MOVING 1
#define SIZE_MX64_MOVING_STATUS 1
#define SIZE_MX64_PRESENT_PWM 2
#define SIZE_MX64_PRESENT_LOAD 2
#define SIZE_MX64_PRESENT_VELOCITY 4
#define SIZE_MX64_PRESENT_POSITION 4
#define SIZE_MX64_VELOCITY_TRAJECTORY 4
#define SIZE_MX64_POSITION_TRAJECTORY 4
#define SIZE_MX64_PRESENT_INPUT_VOLTAGE 2
#define SIZE_MX64_PRESENT_TEMPERATURE 1


namespace Darwin {
/**
 * @brief This represents, and gives access to the darwin's MX106 motors
 *
 * @author Trent Houliston
 */
class MX106 : public DarwinDevice {

public:
    /**
     * @brief This enum holds the addresses of the various bytes in the MX106.
     *
     * @details
     *  for additional details see http://support.robotis.com/en/product/dynamixel/mx_series/mx-28.htm
     */

    enum {
        ADD_MX64_MODEL_NUMBER           = 0,
        ADD_MX64_MODEL_INFORMATION      = 2,
        ADD_MX64_VERSION_OF_FIRMWARE    = 6,
        ADD_MX64_ID                     = 7,
        ADD_MX64_BAUD_RATE              = 8,
        ADD_MX64_RETURN_DELAY_TIME      = 9,
        ADD_MX64_DRIVE_MODE             = 10,
        ADD_MX64_OPERATING_MODE         = 11,
        ADD_MX64_SECONDARY_SHADOW_ID    = 12,
        ADD_MX64_PROTOCOL_VERSION       = 13,
        ADD_MX64_HOMING_OFFSET          = 20,
        ADD_MX64_MOVING_THRESHOLD       = 24,
        ADD_MX64_TEMPERATURE_LIMIT      = 31,
        ADD_MX64_MAX_VOLTAGE_LIMIT      = 32,
        ADD_MX64_MIN_VOLTAGE_LIMIT      = 34,
        ADD_MX64_PWM_LIMIT              = 36,
        ADD_MX64_CURRENT_LIMIT          = 38,
        ADD_MX64_ACCELERATION_LIMIT     = 40,
        ADD_MX64_VELOCITY_LIMIT         = 44,
        ADD_MX64_MAX_POSITION_LIMIT     = 48,
        ADD_MX64_MIN_POSITION_LIMIT     = 52,
        ADD_MX64_SHUTDOWN               = 63,
        ADD_MX64_TORQUE_ENABLE          = 64,
        ADD_MX64_LED                    = 65,
        ADD_MX64_STATUS_RETURN_LEVEL    = 68,
        ADD_MX64_REGISTERED_INSTRUCTION = 69,
        ADD_MX64_HARDWARE_ERROR_STATUS  = 70,
        ADD_MX64_VELOCITY_I_GAIN        = 76,
        ADD_MX64_VELOCITY_P_GAIN        = 78,
        ADD_MX64_POSITION_D_GAIN        = 80,
        ADD_MX64_POSITION_I_GAIN        = 82,
        ADD_MX64_POSITION_P_GAIN        = 84,
        ADD_MX64_FEEDFORWARD_2ND_GAIN   = 88,
        ADD_MX64_FEEDFORWARD_1ST_GAIN   = 90,
        ADD_MX64_BUS_WATCHDOG           = 98,
        ADD_MX64_GOAL_PWM               = 100,
        ADD_MX64_GOAL_CURRENT           = 102,
        ADD_MX64_GOAL_VELOCITY          = 104,
        ADD_MX64_PROFILE_ACCELERATION   = 108,
        ADD_MX64_PROFILE_VELOCITY       = 112,
        ADD_MX64_GOAL_POSITION          = 116,
        ADD_MX64_REALTIME_TICK          = 120,
        ADD_MX64_MOVING                 = 122,
        ADD_MX64_MOVING_STATUS          = 123,
        ADD_MX64_PRESENT_PWM            = 124,
        ADD_MX64_PRESENT_LOAD           = 126,
        ADD_MX64_PRESENT_VELOCITY       = 128,
        ADD_MX64_PRESENT_POSITION       = 132,
        ADD_MX64_VELOCITY_TRAJECTORY    = 136,
        ADD_MX64_POSITION_TRAJECTORY    = 140,
        ADD_MX64_PRESENT_INPUT_VOLTAGE  = 144,
        ADD_MX64_PRESENT_TEMPERATURE    = 146
    };

    MX64(UART& coms, int id);
};
}  // namespace Darwin

#endif
