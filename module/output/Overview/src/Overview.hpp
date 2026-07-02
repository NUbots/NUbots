/*
 * MIT License
 *
 * Copyright (c) 2021 NUbots
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
#ifndef MODULE_OUTPUT_OVERVIEW_HPP
#define MODULE_OUTPUT_OVERVIEW_HPP

#include <chrono>
#include <nuclear>
#include <string>

namespace module::output {

    class Overview : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the Overview reactor.
        explicit Overview(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct Config {
            /// @brief The period between Overview emissions
            std::chrono::duration<double> send_period{0.5};
            /// @brief Whether to also send the serialised Overview message as a UDP packet
            bool udp_enabled = false;
            /// @brief The IP address to send the UDP Overview packet to
            std::string udp_ip_address = "";
            /// @brief The port to send the UDP Overview packet to
            uint16_t udp_port = 0;
        } cfg;

        /// @brief Whether the periodic send loop has been started yet
        bool send_loop_started = false;

        NUClear::clock::time_point last_camera_image;
        NUClear::clock::time_point last_seen_ball;
        NUClear::clock::time_point last_seen_goal;
    };

}  // namespace module::output

#endif  // MODULE_OUTPUT_OVERVIEW_HPP
