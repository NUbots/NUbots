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

#define SIZE_MX106_Model Number 2
#define SIZE_MX106_Model Information 4
#define SIZE_MX106_Version of Firmware 1
#define SIZE_MX106_ID 1
#define SIZE_MX106_Baud Rate 1
#define SIZE_MX106_Return Delay Time 1
#define SIZE_MX106_Drive Mode 1
#define SIZE_MX106_Operating Mode 1
#define SIZE_MX106_Secondary(Shadow) ID 1
#define SIZE_MX106_Protocol Version 1
#define SIZE_MX106_Homing Offset 4
#define SIZE_MX106_Moving Threshold 4
#define SIZE_MX106_Temperature Limit 1
#define SIZE_MX106_Max Voltage Limit 2
#define SIZE_MX106_Min Voltage Limit 2
#define SIZE_MX106_PWM Limit 2
#define SIZE_MX106_Current Limit 2
#define SIZE_MX106_Acceleration Limit 4
#define SIZE_MX106_Velocity Limit 4
#define SIZE_MX106_Max Position Limit 4
#define SIZE_MX106_Min Position Limit 4
#define SIZE_MX106_Shutdown 1
#define SIZE_MX106_Torque Enable 1
#define SIZE_MX106_LED 1
#define SIZE_MX106_Status Return Level 1
#define SIZE_MX106_Registered Instruction 1
#define SIZE_MX106_Hardware Error Status 1
#define SIZE_MX106_Velocity I Gain 2
#define SIZE_MX106_Velocity P Gain 2
#define SIZE_MX106_Position D Gain 2
#define SIZE_MX106_Position I Gain 2
#define SIZE_MX106_Position P Gain 2
#define SIZE_MX106_Feedforward 2nd Gain 2
#define SIZE_MX106_Feedforward 1st Gain 2
#define SIZE_MX106_Bus Watchdog 1
#define SIZE_MX106_Goal PWM 2
#define SIZE_MX106_Goal Current 2
#define SIZE_MX106_Goal Velocity 4
#define SIZE_MX106_Profile Acceleration 4
#define SIZE_MX106_Profile Velocity 4
#define SIZE_MX106_Goal Position 4
#define SIZE_MX106_Realtime Tick 2
#define SIZE_MX106_Moving 1
#define SIZE_MX106_Moving Status 1
#define SIZE_MX106_Present PWM 2
#define SIZE_MX106_Present Current 2
#define SIZE_MX106_Present Velocity 4
#define SIZE_MX106_Present Position 4
#define SIZE_MX106_Velocity Trajectory 4
#define SIZE_MX106_Position Trajectory 4
#define SIZE_MX106_Present Input Voltage 2
#define SIZE_MX106_Present Temperature 1
#define SIZE_MX106_Indirect Address 1 2
#define SIZE_MX106_Indirect Address 2 2
#define SIZE_MX106_Indirect Address 3 2
#define SIZE_MX106_Indirect Address 26 2
#define SIZE_MX106_Indirect Address 27 2
#define SIZE_MX106_Indirect Address 28 2
#define SIZE_MX106_Indirect Data 1 1
#define SIZE_MX106_Indirect Data 2 1
#define SIZE_MX106_Indirect Data 3 1
#define SIZE_MX106_Indirect Data 26 1
#define SIZE_MX106_Indirect Data 27 1
#define SIZE_MX106_Indirect Data 28 1
#define SIZE_MX106_Indirect Address 29 2
#define SIZE_MX106_Indirect Address 30 2
#define SIZE_MX106_Indirect Address 31 2
#define SIZE_MX106_Indirect Address 54 2
#define SIZE_MX106_Indirect Address 55 2
#define SIZE_MX106_Indirect Address 56 2
#define SIZE_MX106_Indirect Data 29 1
#define SIZE_MX106_Indirect Data 30 1
#define SIZE_MX106_Indirect Data 31 1
#define SIZE_MX106_Indirect Data 54 1
#define SIZE_MX106_Indirect Data 55 1
#define SIZE_MX106_Indirect Data 56 1

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
        ADD_MX106_MODEL_NUMBER           = 0,
        ADD_MX106_MODEL_INFORMATION      = 2,
        ADD_MX106_VERSION_OF_FIRMWARE    = 6,
        ADD_MX106_ID                     = 7,
        ADD_MX106_BAUD_RATE              = 8,
        ADD_MX106_RETURN_DELAY_TIME      = 9,
        ADD_MX106_DRIVE_MODE             = 10,
        ADD_MX106_OPERATING_MODE         = 11,
        ADD_MX106_SECONDARY_SHADOW_ID    = 12,
        ADD_MX106_PROTOCOL_VERSION       = 13,
        ADD_MX106_HOMING_OFFSET          = 20,
        ADD_MX106_MOVING_THRESHOLD       = 24,
        ADD_MX106_TEMPERATURE_LIMIT      = 31,
        ADD_MX106_MAX_VOLTAGE_LIMIT      = 32,
        ADD_MX106_MIN_VOLTAGE_LIMIT      = 34,
        ADD_MX106_PWM_LIMIT              = 36,
        ADD_MX106_CURRENT_LIMIT          = 38,
        ADD_MX106_ACCELERATION_LIMIT     = 40,
        ADD_MX106_VELOCITY_LIMIT         = 44,
        ADD_MX106_MAX_POSITION_LIMIT     = 48,
        ADD_MX106_MIN_POSITION_LIMIT     = 52,
        ADD_MX106_SHUTDOWN               = 63,
        ADD_MX106_TORQUE_ENABLE          = 64,
        ADD_MX106_LED                    = 65,
        ADD_MX106_STATUS_RETURN_LEVEL    = 68,
        ADD_MX106_REGISTERED_INSTRUCTION = 69,
        ADD_MX106_HARDWARE_ERROR_STATUS  = 70,
        ADD_MX106_VELOCITY_I_GAIN        = 76,
        ADD_MX106_VELOCITY_P_GAIN        = 78,
        ADD_MX106_POSITION_D_GAIN        = 80,
        ADD_MX106_POSITION_I_GAIN        = 82,
        ADD_MX106_POSITION_P_GAIN        = 84,
        ADD_MX106_FEEDFORWARD_2ND_GAIN   = 88,
        ADD_MX106_FEEDFORWARD_1ST_GAIN   = 90,
        ADD_MX106_BUS_WATCHDOG           = 98,
        ADD_MX106_GOAL_PWM               = 100,
        ADD_MX106_GOAL_CURRENT           = 102,
        ADD_MX106_GOAL_VELOCITY          = 104,
        ADD_MX106_PROFILE_ACCELERATION   = 108,
        ADD_MX106_PROFILE_VELOCITY       = 112,
        ADD_MX106_GOAL_POSITION          = 116,
        ADD_MX106_REALTIME_TICK          = 120,
        ADD_MX106_MOVING                 = 122,
        ADD_MX106_MOVING_STATUS          = 123,
        ADD_MX106_PRESENT_PWM            = 124,
        ADD_MX106_PRESENT_CURRENT        = 126,
        ADD_MX106_PRESENT_VELOCITY       = 128,
        ADD_MX106_PRESENT_POSITION       = 132,
        ADD_MX106_VELOCITY_TRAJECTORY    = 136,
        ADD_MX106_POSITION_TRAJECTORY    = 140,
        ADD_MX106_PRESENT_INPUT_VOLTAGE  = 144,
        ADD_MX106_PRESENT_TEMPERATURE    = 146,
        ADD_MX106_INDIRECT_ADDRESS_1     = 168,
        ADD_MX106_INDIRECT_ADDRESS_2     = 170,
        ADD_MX106_INDIRECT_ADDRESS_3     = 172,
        ADD_MX106_INDIRECT_ADDRESS_26    = 218,
        ADD_MX106_INDIRECT_ADDRESS_27    = 220,
        ADD_MX106_INDIRECT_ADDRESS_28    = 222,
        ADD_MX106_INDIRECT_DATA_1        = 224,
        ADD_MX106_INDIRECT_DATA_2        = 225,
        ADD_MX106_INDIRECT_DATA_3        = 226,
        ADD_MX106_INDIRECT_DATA_26       = 249,
        ADD_MX106_INDIRECT_DATA_27       = 250,
        ADD_MX106_INDIRECT_DATA_28       = 251,
        ADD_MX106_INDIRECT_ADDRESS_29    = 578,
        ADD_MX106_INDIRECT_ADDRESS_30    = 580,
        ADD_MX106_INDIRECT_ADDRESS_31    = 582,
        ADD_MX106_INDIRECT_ADDRESS_54    = 628,
        ADD_MX106_INDIRECT_ADDRESS_55    = 630,
        ADD_MX106_INDIRECT_ADDRESS_56    = 632,
        ADD_MX106_INDIRECT_DATA_29       = 634,
        ADD_MX106_INDIRECT_DATA_30       = 635,
        ADD_MX106_INDIRECT_DATA_31       = 636,
        ADD_MX106_INDIRECT_DATA_54       = 659,
        ADD_MX106_INDIRECT_DATA_55       = 660,
        ADD_MX106_INDIRECT_DATA_56       = 661
    };

    MX64(UART& coms, int id);
};
}  // namespace Darwin

#endif
