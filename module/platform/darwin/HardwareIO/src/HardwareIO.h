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
         * This NUClear Reactor is responsible for reading in the data for the Darwin Platform and emitting it to the
         * rest
         * of the system
         *
         * @author Trent Houliston
         */
        class HardwareIO : public NUClear::Reactor {
        private:
            // How often we read the servos
            static constexpr int UPDATE_FREQUENCY = 90;

            /// @brief Our internal darwin class that is used for interacting with the hardware
            Darwin::Darwin darwin;
            message::platform::darwin::DarwinSensors parseSensors(const Darwin::BulkReadResults& data);
            float dGain = 0;
            float iGain = 0;
            float pGain = 0;


            struct CM740State {
                message::platform::darwin::DarwinSensors::LEDPanel ledPanel = {false, false, false};
                //  0x00, 0xRR, 0xGG, 0xBB
                message::platform::darwin::DarwinSensors::HeadLED headLED = {0x0000FF00};
                message::platform::darwin::DarwinSensors::EyeLED eyeLED   = {0x000000FF};
            };

            struct Config {
                Config() : battery() {}

                struct Battery {
                    Battery() : chargedVoltage(0.0f), nominalVoltage(0.0f), flatVoltage(0.0f) {}
                    float chargedVoltage;
                    float nominalVoltage;
                    float flatVoltage;
                } battery;
            } config;

            struct ServoState {
                // True if we need to write new values to the hardware
                bool dirty = false;

                // True if we simulate where we think the servos should be
                // Note that we still write the commands to hardware
                bool simulated = false;

                bool torqueEnabled = true;

                // Cached values that are never read
                float pGain        = 32.0 / 255.0;
                float iGain        = 0;
                float dGain        = 0;
                float movingSpeed  = 0;
                float goalPosition = 0;
                float torque       = 0;  // 0.0 to 1.0

                // Values that are either simulated or read
                float presentPosition = 0;
                float presentSpeed    = 0;
                float load            = 0;
                float voltage         = 10;
                float temperature     = 40;
            };

            /// @brief Our state for our CM740 for variables we send to it
            CM740State cm740State;

            /// @brief Our state for or MX28s for variables we send to it
            std::array<ServoState, 20> servoState;

            float chargedVoltage;
            float flatVoltage;

        public:
            /// @brief called by a Powerplant to construct this reactor
            explicit HardwareIO(std::unique_ptr<NUClear::Environment> environment);
        };
    }  // namespace darwin
}  // namespace platform
}  // namespace module
#endif
