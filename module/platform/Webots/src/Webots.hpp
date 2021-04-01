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

#include "message/platform/webots/messages.hpp"
#include "message/platform/webots/ConnectRequest.hpp"

namespace module::platform {

class Webots : public NUClear::Reactor {
private:
    /// @brief Send the inital message that tells webots who we are. We should do this when we first connect or when we
    /// want to reconnect.
	/// @param The socket we want to send our details to.
    void send_connect(int& fd);

    /// @brief A single message that stores all the data that webots needs from us.
    message::platform::webots::ActuatorRequests to_send;

	/// @ brief The information we got from Global Config
	message::platform::webots::ConnectRequest player_details;
	
public:
    /// @brief Called by the powerplant to build and setup the webots reactor.
    explicit Webots(std::unique_ptr<NUClear::Environment> environment);

    /// @brief Establish a TCP connection to the specified server/port
    /// @param server_name The name or IP address to connect to. If it's an IP, it should be in "X.X.X.X" form
    /// @param port The port number to connect to
    /// @return If the connection was successful, a file descriptor. Else, -1 is returned
    int tcpip_connect(const std::string& server_name, const char& port);
};

}  // namespace module::platform

#endif  // MODULE_PLATFORM_WEBOTS_HPP
