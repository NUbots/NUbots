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

#ifndef MODULES_PLATFORM_CM740_HARDWAREIO_HPP
#define MODULES_PLATFORM_CM740_HARDWAREIO_HPP

#include <nuclear>
#include <yaml-cpp/yaml.h>

#include "cm740/CM740.hpp"

#include "message/platform/RawSensors.hpp"

namespace module::platform::cm740 {

    /**
     * This NUClear Reactor is responsible for reading in the data from the CM740 and emitting it to
     * the rest of the system
     *
     * @author Trent Houliston
     */
    class HardwareIO : public NUClear::Reactor {
    private:
        /// @brief How often the servos are read.
        static constexpr int UPDATE_FREQUENCY = 90;

        /// @brief Internal cm740 class that is used for interacting with the hardware.
        CM740::CM740 cm740{"/dev/CM740"};

        /// @brief Reads information from CM740 packet and processes it into a RawSensors message.
        /// @param data CM740 packet information.
        /// @return A RawSensors message created from the data sent by the CM740 subcontroller.
        message::platform::RawSensors parseSensors(const CM740::BulkReadResults& data);
        /// @brief Derivative gain of the servos.
        float d_gain = 0.0f;
        /// @brief Integral gain of the servos.
        float i_gain = 0.0f;
        /// @brief Proportional gain of the servos.
        float p_gain = 0.0f;

        /// @brief State of the CM740 LEDs, including the LED panel, head LED and eye LED.
        struct LEDState {
            /// @brief Booleans representing which of the three LED panel lights are on
            message::platform::RawSensors::LEDPanel led_panel = {false, false, false};
            /// @brief Colour of the head LED, of the form 0x00RRGGBB
            message::platform::RawSensors::HeadLED head_LED = {0x0000FF00};
            /// @brief Colour of the eye LED, of the form 0x00RRGGBB
            message::platform::RawSensors::EyeLED eye_LED = {0x000000FF};
        } led_state;

        /// @brief Configuration values
        struct Config {
            Config() = default;
            /// @brief Battery voltage information.
            struct Battery {
                Battery() = default;
                float charged_voltage{0.0f};
                float nominal_voltage{0.0f};
                float flat_voltage{0.0f};
            } battery;
        } cfg;

        struct ServoState {
            /// @brief True if new values should be written to the hardware.
            bool dirty = false;

            /// @brief True if servo positions are simulated.
            /// @details Note that commands can still be written to the hardware.
            bool simulated = false;

            /// @brief Whether the servo torque is enabled.
            bool torque_enabled = true;

            /// @brief Proportional gain. Cached value that is never read.
            float p_gain = 32.0 / 255.0;
            /// @brief Integral gain. Cached value that is never read.
            float i_gain = 0.0f;
            /// @brief Derivative gain. Cached value that is never read.
            float d_gain = 0.0f;
            /// @brief Speed of the servo. Cached value that is never read.
            float moving_speed = 0.0f;
            /// @brief Position the servo is attempting to reach. Cached value that is never read.
            float goal_position = 0.0f;
            /// @brief Torque of the servo. Cached value that is never read.
            /// @details Ranges from 0.0 to 1.0.
            float torque = 0.0f;

            /// @brief Current angle position of the servo. Simulated or read.
            float present_position = 0.0f;
            /// @brief Current speed of the servo. Simulated or read.
            float present_speed = 0.0f;
            /// @brief Current load of the servo. Simulated or read.
            float load = 0.0f;
            /// @brief Current voltage of the servo. Simulated or read.
            float voltage = 10.0f;
            /// @brief Current temperature of the servo. Simulated or read.
            float temperature = 40.0f;
        };

        /// @brief Stores the current state of the servos and the data being sent to the servos.
        std::array<ServoState, 20> servo_state;

    public:
        /// @brief called by a Powerplant to construct this reactor
        explicit HardwareIO(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::platform::cm740
#endif
