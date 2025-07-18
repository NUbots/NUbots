/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef MODULE_NETWORK_ROBOTCOMMUNICATION_HPP
#define MODULE_NETWORK_ROBOTCOMMUNICATION_HPP

#include <nuclear>
#include <string>

namespace module::network {

    class RobotCommunication : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief The port to communicate over
            uint send_port = 0;
            /// @brief The port to communicate over (should match receive port)
            uint receive_port = 0;
            /// @brief The IP address used for broadcasting data
            std::string broadcast_ip = "";
            /// @brief Set this to only receive packets from this IP address
            std::string udp_filter_address = "";
            /// @brief A delay before the first message is sent, to allow reasonable data to be collected
            int startup_delay = 0;
            /// @brief The timeout for the ball position to be used
            std::chrono::seconds ball_timeout{0};
        } cfg;

        /// @brief ignore packets from these IP addresses
        std::set<std::string> ignored_ip_addresses{};

        /// @brief Handle for incoming UDP message. This will be bound/unbound during (re)connection
        ReactionHandle listen_handle;

    public:
        /// @brief Called by the powerplant to build and setup the RobotCommunication reactor.
        explicit RobotCommunication(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::network

#endif  // MODULE_NETWORK_ROBOTCOMMUNICATION_HPP
