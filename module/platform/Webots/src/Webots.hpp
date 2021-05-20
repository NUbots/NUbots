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
 * Copyright 2021 NUbots <nubots@nubots.net>
 */

#ifndef MODULE_PLATFORM_WEBOTS_HPP
#define MODULE_PLATFORM_WEBOTS_HPP

#include <nuclear>

#include "message/platform/webots/ConnectRequest.hpp"
#include "message/platform/webots/messages.hpp"

namespace module::platform {

    class Webots : public NUClear::Reactor {
    private:
        // How often we read the servos
        static constexpr int UPDATE_FREQUENCY = 90;

        /// @brief Handle for incoming data reaction. This will be bound/unbound during (re)connection
        ReactionHandle read_io;
        /// @brief Handle for outgoing data reaction. This will be bound/unbound during (re)connection
        ReactionHandle send_io;
        /// @brief Handle for error checking on the TCP connection. This will be bound/unbound during (re)connection
        ReactionHandle error_io;

        /// @brief Establish a TCP connection to the specified server/port
        /// @param server_name The name or IP address to connect to. If it's an IP, it should be in "X.X.X.X" form
        /// @param port The port number to connect to
        /// @return If the connection was successful, a file descriptor. Else, -1 is returned
        int tcpip_connect(const std::string& server_name, const std::string& port);

        /// @brief Establishes the connection with webots, then binds the reaction handles with the resulting fd
        /// @param server_address The IP address to connect to, in "X.X.X.X" form
        /// @param port The port number to connect to
        void setup_connection(const std::string& server_address, const std::string& port);

        /// @brief Translate sensor measurement messages Webots sends us, emmitting readings as our message types
        /// @param sensor_measurements Message from Webots with information from the sensors
        void translate_and_emit_sensor(const message::platform::webots::SensorMeasurements& sensor_measurements);

        /// @brief The current file descriptor used for the connection. It should be kept -1 if no active connection
        int fd;

        /// @brief The time the connection was opened.
        NUClear::clock::time_point connect_time;

        /// @brief The number of time ticks which have passed since the last IO::READ trigger
        uint32_t sim_delta = 0;
        /// @brief The number of milliseconds which have passed since the last IO::READ trigger
        uint64_t real_delta = 0;
        /// @brief The current simulation time in ticks
        uint32_t current_sim_time = 0;
        /// @brief The current real time in milliseconds (unix time)
        uint64_t current_real_time = 0;

        /// @brief The time between two measurements, expressed in milliseconds
        int time_step;

        /// @brief Current state of a servo
        struct ServoState {
            /// @brief True if we need to write new values to the simulator
            bool dirty = false;

            /// @brief ID of the servo
            int id;

            /// @brief Name of the servo
            std::string name;

            /// @brief Cached values that are never read from the simulator
            float p_gain        = 32.0f / 255.0f;
            float i_gain        = 0.0f;
            float d_gain        = 0.0f;
            float moving_speed  = 0.0f;
            float goal_position = 0.0f;
            float torque        = 0.0f;  // 0.0 to 1.0

            /// Values that are read from the simulator
            float present_position = 0.0f;
            float present_speed    = 0.0f;
        };

        /// @brief Our current servo states
        std::array<ServoState, 20> servo_state;

    public:
        /// @brief Called by the powerplant to build and setup the webots reactor
        explicit Webots(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::platform

#endif  // MODULE_PLATFORM_WEBOTS_HPP
