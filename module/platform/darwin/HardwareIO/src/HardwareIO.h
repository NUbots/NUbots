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

#ifndef MODULES_PLATFORM_DARWIN_HARDWAREIO_H
#define MODULES_PLATFORM_DARWIN_HARDWAREIO_H

#include <nuclear>

#include <yaml-cpp/yaml.h>

#include "darwin/Darwin.h"
#include "message/platform/darwin/DarwinSensors.h"

namespace module {
namespace platform {
namespace darwin {

    /**
     * This NUClear Reactor is responsible for reading in the data for the Darwin Platform and emitting it to the rest
     * of the system
     *
     * @author Trent Houliston
     */
    class HardwareIO : public NUClear::Reactor {
    private:
        /// @brief Our internal darwin class that is used for interacting with the hardware
        Darwin::Darwin darwin;
        message::platform::darwin::DarwinSensors parseSensors(const Darwin::BulkReadResults& data);
        float dGain = 0;
        float iGain = 0;
        float pGain = 0;


        struct CM730State {
            message::platform::darwin::DarwinSensors::LEDPanel ledPanel = { false, false, false };
                                                                        //  0x00, 0xRR, 0xGG, 0xBB
            message::platform::darwin::DarwinSensors::HeadLED headLED   = { 0x0000FF00 };
            message::platform::darwin::DarwinSensors::EyeLED eyeLED     = { 0x000000FF };
        };

        struct ServoState {
            bool dirty = false;

            bool torqueEnabled = true;

            float pGain = 32.0/255.0;
            float iGain = 0;
            float dGain = 0;
            float torque = 0; // 0.0 to 1.0
            float movingSpeed = 0;
            float goalPosition = 0;

        };

        /// @brief Our state for our CM730 for variables we send to it
        CM730State cm730State;

        /// @brief Our state for or MX28s for variables we send to it
        std::array<ServoState, 20> servoState;

    public:
        /// @brief called by a Powerplant to construct this reactor
        explicit HardwareIO(std::unique_ptr<NUClear::Environment> environment);
    };
}
}
}
#endif

