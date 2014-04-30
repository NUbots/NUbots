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

#include "HardwareIO.h"
#include "Convert.h"

#include "messages/platform/darwin/DarwinSensors.h"
#include "messages/motion/ServoTarget.h"


namespace modules {
namespace platform {
namespace darwin {

    using messages::platform::darwin::DarwinSensors;
    using messages::motion::ServoTarget;

    HardwareIO::HardwareIO(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), darwin("/dev/ttyUSB0") {

        // This trigger gets the sensor data from the CM730
        on<Trigger<Every<60, Per<std::chrono::seconds>>>, Options<Single>>([this](const time_t&) {

            // Read our data and convert it into standard format
            auto sensors = std::make_unique<DarwinSensors>(
                converter.convert(darwin.bulkRead()));

            // Emit it to the rest of the system
            emit(std::move(sensors));
        });

        // This trigger writes the servo positions to the hardware
        on<Trigger<std::vector<ServoTarget>>, With<DarwinSensors>>([this](const std::vector<ServoTarget>& commands, const DarwinSensors& sensors) {

            std::vector<Darwin::Types::ServoValues> values;

            // Loop through each of our commands
            for (const auto& command : commands) {

                // If gain is nan, do a normal write to disable torque (syncwrite won't write to torqueEnable)
                if(isnan(command.gain)) {
                    darwin[static_cast<int>(command.id) + 1].write(Darwin::MX28::Address::TORQUE_ENABLE, false);
                }

                // Otherwise write the command using sync write
                else {
                    values.push_back(convert(command));
                }
            }

            // Syncwrite our values
            darwin.writeServos(values);
        });

        on<Trigger<ServoTarget>>([this](const ServoTarget command) {
            auto commandList = std::make_unique<std::vector<ServoTarget>>();
            commandList->push_back(command);

            // Emit it so it's captured by the reaction above
            emit<Scope::DIRECT>(std::move(commandList));
        });

        // If we get a HeadLED command then write it
        on<Trigger<DarwinSensors::HeadLED>>([this](const DarwinSensors::HeadLED& led) {
            darwin.cm730.write(Darwin::CM730::Address::LED_HEAD_L, Convert::colourLEDInverse(led.r, led.g, led.b));
        });

        // If we get a HeadLED command then write it
        on<Trigger<DarwinSensors::EyeLED>>([this](const DarwinSensors::EyeLED& led) {
            darwin.cm730.write(Darwin::CM730::Address::LED_EYE_L, Convert::colourLEDInverse(led.r, led.g, led.b));
        });
    }
}
}
}
