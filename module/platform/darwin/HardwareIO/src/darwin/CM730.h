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

#ifndef DARWIN_CM730_H_
#define DARWIN_CM730_H_

#include "DarwinDevice.h"

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
         *  for additional details see http://support.robotis.com/en/product/darwin-op/references/reference/hardware_specifications/electronics/sub_controller_(cm-730).htm
         */
        enum Address {
            // CM730 ROM register addresses (standard format for all Dynamixel bus devices)
            MODEL_NUMBER_L       = 0,  // Model number (L)
            MODEL_NUMBER_H       = 1,  // Model Number (H)
            VERSION              = 2,  // Firmware version
            ID                   = 3,  // Device ID on Dynamixel bus
            BAUD_RATE            = 4,  // Baud rate
            RETURN_DELAY_TIME    = 5,  // Return delay time
            CW_ANGLE_LIMIT_L     = 6,  // Clockwise angle limit (L)
            CW_ANGLE_LIMIT_H     = 7,  // Clockwise angle limit (H)
            CCW_ANGLE_LIMIT_L    = 8,  // Counterclockwise angle limit (L)
            CCW_ANGLE_LIMIT_H    = 9,  // Counterclockwise angle limit (H)
            SYSTEM_DATA2         = 10, // MX106 => Drive mode, MX64 => Reserved
            LIMIT_TEMPERATURE    = 11, // Internal limit temperature
            VOLTAGE_LOWER_LIMIT  = 12, // Voltage lower limit
            VOLTAGE_UPPER_LIMIT  = 13, // Voltage upper limit
            MAX_TORQUE_L         = 14, // Maximum torque (L)
            MAX_TORQUE_H         = 15, // Maximum torque (H)
            RETURN_LEVEL         = 16, // Status return level
            ALARM_LED            = 17, // Alarm LED (this register is used for the CM730 to signal error conditions, and not the ALARM_SHUTDOWN one)
            ALARM_SHUTDOWN       = 18, // Alarm shutdown
            OPERATING_MODE       = 19, // Custom     (Bonn-spcific??)
            DOWN_CALIBRATION_L   = 20, // Custom     (Bonn-spcific??)
            DOWN_CALIBRATION_H   = 21, // Custom     (Bonn-spcific??)
            UP_CALIBRATION_L     = 22, // Custom     (Bonn-spcific??)
            UP_CALIBRATION_H     = 23, // Custom     (Bonn-spcific??)

            // CM730 RAM register addresses
            DXL_POWER            = 24,
            LED_PANEL            = 25,
            LED_HEAD_L           = 26,   // RGB LED 5
            LED_HEAD_H           = 27,
            LED_EYE_L            = 28,   // RGB LED 6
            LED_EYE_H            = 29,
            BUTTON               = 30,
            BATTERY_VOLTAGE      = 31,
            GYRO_X_L             = 32,
            GYRO_X_H             = 33,
            GYRO_Y_L             = 34,
            GYRO_Y_H             = 35,
            GYRO_Z_L             = 36,
            GYRO_Z_H             = 37,
            ACCEL_X_L            = 38,
            ACCEL_X_H            = 39,
            ACCEL_Y_L            = 40,
            ACCEL_Y_H            = 41,
            ACCEL_Z_L            = 42,
            ACCEL_Z_H            = 43,
            MAG_X_L              = 44,
            MAG_X_H              = 45,
            MAG_Y_L              = 46,
            MAG_Y_H              = 47,
            MAG_Z_L              = 48,
            MAG_Z_H              = 49,
            TEMPERATURE          = 50,
            BATTERY_L            = 51,   // ADC0
            BATTERY_H            = 52,
            LEFT_MIC_L           = 53,   // ADC1 (MIC1)
            LEFT_MIC_H           = 54,
            ADC2_L               = 55,   // Bonn list this as MIC2
            ADC2_H               = 56,
            ADC3_L               = 57,
            ADC3_H               = 58,
            ADC4_L               = 59,
            ADC4_H               = 60,
            ADC5_L               = 61,
            ADC5_H               = 62,
            ADC6_L               = 63,
            ADC6_H               = 64,
            ADC7_L               = 65,
            ADC7_H               = 66,
            ADC8_L               = 67,
            ADC8_H               = 68,
            RIGHT_MIC_L          = 69,   // ADC9
            RIGHT_MIC_H          = 70,
            ADC10_L              = 71,
            ADC10_H              = 72,
            ADC11_L              = 73,
            ADC11_H              = 74,
            ADC12_L              = 75,
            ADC12_H              = 76,
            ADC13_L              = 77,
            ADC13_H              = 78,
            ADC14_L              = 79,
            ADC14_H              = 80,
            ADC15_L              = 81,
            ADC15_H              = 82,
            BUZZER_PLAY_LENGTH   = 83,
            BUZZER_DATA          = 84,
            ZIGBEE_ID            = 85,
            TX_REMOCON_DATA_L    = 86,
            TX_REMOCON_DATA_H    = 87,
            RX_REMOCON_DATA_L    = 88,
            RX_REMOCON_DATA_H    = 89,
            RX_REMOCON_DATA_ARR  = 90,
            DXLRX_PACKET_CNT_L   = 91,
            DXLRX_PACKET_CNT_H   = 91,
            DXLRX_OVERFLOW_CNT_L = 93,
            DXLRX_OVERFLOW_CNT_H = 94,
            DXLRX_BUFERROR_CNT_L = 95,
            DXLRX_BUFERROR_CNT_H = 96,
            DXLRX_CHKERROR_CNT_L = 97,
            DXLRX_CHKERROR_CNT_H = 98,
            DXLRX_ORE_CNT_L      = 99,
            DXLRX_ORE_CNT_H      = 100,
            DXLTX_PACKET_CNT_L   = 101,
            DXLTX_PACKET_CNT_H   = 102,
            DXLTX_OVERFLOW_CNT_L = 103,
            DXLTX_OVERFLOW_CNT_H = 104,
            DXLTX_BUFERROR_CNT_L = 105,
            DXLTX_BUFERROR_CNT_H = 106,
            PCRX_PACKET_CNT_L    = 107,
            PCRX_PACKET_CNT_H    = 108,
            PCRX_OVERFLOW_CNT_L  = 109,
            PCRX_OVERFLOW_CNT_H  = 110,
            PCRX_BUFERROR_CNT_L  = 111,
            PCRX_BUFERROR_CNT_H  = 112,
            PCRX_CHKERROR_CNT_L  = 113,
            PCRX_CHKERROR_CNT_H  = 114,
            PCRX_ORE_CNT_L       = 115,
            PCRX_ORE_CNT_H       = 116,
            PCTX_PACKET_CNT_L    = 117,
            PCTX_PACKET_CNT_H    = 118,
            PCTX_OVERFLOW_CNT_L  = 119,
            PCTX_OVERFLOW_CNT_H  = 120,
            PCTX_BUFERROR_CNT_L  = 121,
            PCTX_BUFERROR_CNT_H  = 122,
            MISC0                = 123,
            MISC1                = 124,
            MISC2                = 125,
            MISC3                = 126
        };

        CM730(UART& coms, int id);

        /**
         * @brief turns on the Dynamixel power (servos and foot sensors)
         */
        void turnOnDynamixel();
    };
}  // namespace Darwin

#endif
