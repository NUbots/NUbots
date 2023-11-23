/*
 * MIT License
 *
 * Copyright (c) 2022 NUbots
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
#ifndef MODULE_NETWORK_PLOTJUGGLER_HPP
#define MODULE_NETWORK_PLOTJUGGLER_HPP

#include <nuclear>
#include <string>

namespace module::network {

    class PlotJuggler : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the PlotJuggler reactor.
        explicit PlotJuggler(std::unique_ptr<NUClear::Environment> environment);

    private:
        /// @brief The reactor start time in milliseconds since Epoch.
        /// Used to normalise timestamps in forwarded DataPoint messages.
        long start_time_ms;

        /// @brief The robot's current hostname (included in data sent, to enable filtering)
        std::string hostname;

        /// @brief The IP address of the PlotJuggler UDP server
        std::string send_address;

        /// @brief The port of the PlotJuggler UDP server
        int send_port;

        /// @brief Handle to the reaction that forwards DataPoints to PlotJuggler
        ReactionHandle forwarder_reaction{};

        /// @brief Handle to the reaction that generates debug waves for testing PlotJuggler connection
        ReactionHandle debug_waves_reaction{};

        /// @brief Convert the given timepoint to a count of milliseconds since Epoch
        static inline long toMillisecondsSinceEpoch(NUClear::clock::time_point timepoint) {
            return std::chrono::time_point_cast<std::chrono::milliseconds>(timepoint).time_since_epoch().count();
        }
    };

}  // namespace module::network

#endif  // MODULE_NETWORK_PLOTJUGGLER_HPP
