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

#include "DarwinPlatform.h"

#include <armadillo>

#include "messages/DarwinServos.h"
#include "messages/DarwinSensors.h"

namespace modules {

    // TODO this contains the conversion factors from raw values into SI units
    namespace Convert {

        // 0 is -4g, 512 is 0g, 1023 is +4g
        //const double ACCELERONOMETER;

        // 0 is -1600 degrees/second 512 is 0 degrees per second, 1023 is 1600 degrees/second
        // 512 * x = 80PI/9 (1600 degrees in radians)
        //const double GYROSCOPE;
    };

    // TODO populate this with angle offsets from the darwin servos to
    namespace Offsets {

    }

    // TODO populate this with -1 and 1 based on the direction the servo goes in "Standard Space"
    namespace Directions {

    }

    DarwinPlatform::DarwinPlatform(NUClear::PowerPlant& plant) : Reactor(plant), darwin("/dev/ttyUSB0") {

        // This trigger gets the sensor data from the CM730
        on<Trigger<Every<20, std::chrono::milliseconds>>>([this](const time_t& time) {

            /*
            // Read our data
            Darwin::BulkReadResults data = darwin.bulkRead();

            Messages::DarwinSensors* sensors = new Messages::DarwinSensors;


             CM730 Data


            // LED Panel
            sensors->ledPanel = {
                (data.cm730.ledPanel & 0x01) == 0x01,
                (data.cm730.ledPanel & 0x02) == 0x02,
                (data.cm730.ledPanel & 0x04) == 0x04
            };

            // Coloured Head LED
            sensors->headLED = {
                // (Swap from BGR to RGB and promote to 24bit)
                static_cast<uint8_t>((data.cm730.headLED & 0x001F) << 3),
                static_cast<uint8_t>((data.cm730.headLED & 0x03E0) >> 2),
                static_cast<uint8_t>((data.cm730.headLED & 0x7C00) >> 7)
            };

            // Coloured Eye LED
            sensors->eyeLED = {
                // (Swap from BGR to RGB and promote to 24bit)
                static_cast<uint8_t>((data.cm730.eyeLED & 0x001F) << 3),
                static_cast<uint8_t>((data.cm730.eyeLED & 0x03E0) >> 2),
                static_cast<uint8_t>((data.cm730.eyeLED & 0x7C00) >> 7)
            };

            // Buttons
            sensors->buttons = {
                (data.cm730.buttons & 0x02) == 0x02,
                (data.cm730.buttons & 0x01) == 0x01
            };

            // Voltage (in volts)
            sensors->voltage = data.cm730.voltage / 10.0;

            // Acceleronometer (in m/s^2)
            sensors->acceleronometer = {
                (data.cm730.acceleronometer.x - 512) * Convert::ACCELERONOMETER,
                (data.cm730.acceleronometer.y - 512) * Convert::ACCELERONOMETER,
                (data.cm730.acceleronometer.z - 512) * Convert::ACCELERONOMETER
            }

            // Gyroscope (in radians/second)
            sensors->gyroscope = {
                (data.cm730.gyroscope.x - 512) * Convert::GYROSCOPE,
                (data.cm730.gyroscope.x - 512) * Convert::GYROSCOPE,
                (data.cm730.gyroscope.x - 512) * Convert::GYROSCOPE
            }

            // TODO before every read, check to see if all the values are 0xFF, this means an error


             Force Sensitive Resistor Data


            // Right FSR
            sensors->fsr = {
                // Right FSR
                {
                    // FSR Values in Newtons
                    data.fsr[0].fsr1 / 1000,
                    data.fsr[0].fsr2 / 1000,
                    data.fsr[0].fsr3 / 1000,
                    data.fsr[0].fsr4 / 1000,

                    // X and Y Centres (or NAN if the foot is off the ground)
                    data.fsr[0].centreX == 0xFF ? NAN : data.fsr[0].centreX / 254.0,
                    data.fsr[0].centreY == 0xFF ? NAN : data.fsr[0].centreY / 254.0
                },
                // Left FSR
                {
                    // FSR Values in Newtons
                    data.fsr[1].fsr1 / 1000,
                    data.fsr[1].fsr2 / 1000,
                    data.fsr[1].fsr3 / 1000,
                    data.fsr[1].fsr4 / 1000,

                    // X and Y Centres (or NAN if the foot is off the ground)
                    data.fsr[1].centreX == 0xFF ? NAN : data.fsr[1].centreX / 254.0,
                    data.fsr[1].centreY == 0xFF ? NAN : data.fsr[1].centreY / 254.0
                }
            };


             Servos




            // TODO we are using the better servos, and have 4096 range rather then 1024

            bool motorTorqueEnabled = data.servos[0].torqueEnabled;
            bool motorLED = data.servos[0].LED;

            float dGain = data.servos[0].dGain / 254.0;
            float iGain = data.servos[0].iGain / 254.0;
            float pGain = data.servos[0].pGain / 254.0;

            // In radians
            // 4096 is max, 0 is min
            float goalPosition = data.servos[0].goalPosition * POSITION_CONVERSION;

            // In radians per second
            const double SPEED_CONVERSION = 0.01193805208; // 0.114 revolutions per minute in radians per second
            float movingSpeed = data.servos[0].movingSpeed * 0.01193805208;

            // Normalized between 0 and 1
            float torqueLimit = data.servos[0].torqueLimit / 1023.0;

            // In radians
            float position = data.servos[0].position * POSITION_CONVERSION;

            // In radians per second Positive is clockwise
            float presentSpeed = ((data.servos[0].speed & 0x3FF) * SPEED_CONVERSION) * (((data.servos[0].speed & 0x400) == 0x400) ? -1 : 1);

            // Normalized between -1 and 1 Positive is clockwise
            float presentLoad = ((data.servos[0].load & 0x3FF) / 1023.0) * (((data.servos[0].load & 0x400) == 0x400) ? -1 : 1);

            // In volts
            float motorVoltage = data.servos[0].voltage / 10.0;

            // In degrees c
            uint8_t temperature = data.servos[0].temperature;
             */
        });

        // This trigger writes the servo positions to the hardware
        on<Trigger<Every<20, std::chrono::milliseconds>>, With<Messages::DarwinServos>>([](const time_t& time, const Messages::DarwinServos& servos) {

            // TODO convert DarwinMotors into our motor type

            std::vector<Darwin::Types::ServoValues> values;

            values.push_back({
                0, // Motor ID
                0 * 254, // D Gain
                0 * 254, // I Gain
                0 * 254, // P Gain
                0x00,   // Reserved Byte
                0 ,// / POSITION_CONVERSION, // Goal Position
                0 ,// / SPEED_CONVERSION, // TODO factor in movement direction
            });
        });
    }
}
