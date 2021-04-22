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

#ifndef MODULE_PLATFORM_WEBOTS_WEBOTS_HPP
#define MODULE_PLATFORM_WEBOTS_WEBOTS_HPP

#include <nuclear>

#include "message/platform/webots/ConnectRequest.hpp"
#include "message/platform/webots/messages.hpp"
#include "message/support/GlobalConfig.hpp"

namespace module::platform::webots {

    class Webots : public NUClear::Reactor {
    private:
        /// @brief A single message that stores all the data that webots needs from us.
        message::platform::webots::ActuatorRequests to_send;

        /// @brief Store a handle to the io reaction so we can unbind it
        ReactionHandle read_io;
        /// @brief Store a handle to the every reaction so we can unbind it
        ReactionHandle send_loop;
        /// @brief Store a handle to the io error reaction so we can unbind it
        ReactionHandle error_io;
        /// @brief Store a handle to the shutdown reaction so we can unbind it
        ReactionHandle shutdown_handle;

        /// @brief Send the inital message that tells webots who we are. We should do this when we first connect or when
        /// we want to reconnect.
        /// @param fd The socket we want to send our details to.
        void send_player_details(const int& fd, const message::support::GlobalConfig& player_details);

        /// @brief Establish a TCP connection to the specified server/port
        /// @param server_name The name or IP address to connect to. If it's an IP, it should be in "X.X.X.X" form
        /// @param port The port number to connect to
        /// @return If the connection was successful, a file descriptor. Else, -1 is returned
        int tcpip_connect(const std::string& server_name, const std::string& port);

        void setup_connection(const std::string& server_address, const std::string& port);

        void translate_and_emit_sensor(const message::platform::webots::SensorMeasurements& sensor_measurements);

        /// @brief The time the connection was opened.
        NUClear::clock::time_point connect_time;

    public:
        /// @brief Called by the powerplant to build and setup the webots reactor.
        explicit Webots(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::platform::webots

#endif  // MODULE_PLATFORM_WEBOTS_HPP
