/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
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

#ifndef MODULE_PLATFORM_NUSENSE_HARDWAREIO_HPP
#define MODULE_PLATFORM_NUSENSE_HARDWAREIO_HPP

#include <array>
#include <nuclear>
#include <string>

#include "NUSenseParser.hpp"
#include "NUgus.hpp"

#include "utility/reactor/StreamReactor.hpp"

namespace module::platform::NUSense {

    class HardwareIO : public utility::reactor::StreamReactor<HardwareIO, NUSenseParser, 5> {
    public:
        /// @brief Called by the powerplant to build and setup the HardwareIO reactor.
        explicit HardwareIO(std::unique_ptr<NUClear::Environment> environment);

    private:
        /// @brief  Configuration variables for this reactor
        struct Config {
            struct {
                /// @brief The port to connect to the NUSense device
                std::string port = "";
                /// @brief The baud rate to communication with the NUSense device
                int baud = 0;
            } nusense{};
        } cfg{};

        /// @brief Contains device information specific to the NUgus robot
        NUgus nugus{};

    private:
        /// @brief Send a TransmitData message containing an nbs packet to StreamReactor so it can write the data to
        /// NUSense.
        /// @tparam T is the type of the protobuf message to be serialised. In NUSense's case, we expect this to be
        /// SubcontrollerServoTargets.
        /// @param packet A const reference to a protobuf message of type T. This gets serialised and turned to a
        /// vector of bytes before sending to NUSense.
        template <typename T>
        void send_packet(const T& packet) {
            // Serialise the packet
            auto payload = NUClear::util::serialise::Serialise<T>::serialise(packet);
            // Get the hash of the packet
            uint64_t hash = NUClear::util::serialise::Serialise<T>::hash();

            // Get the timestamp of the emit if we can, otherwise use now
            const auto* task = NUClear::threading::ReactionTask::get_current_task();
            auto timestamp = task ? task->stats ? task->stats->emitted : NUClear::clock::now() : NUClear::clock::now();
            auto timestamp_us =
                std::chrono::duration_cast<std::chrono::microseconds>(timestamp.time_since_epoch()).count();
            uint32_t size = uint32_t(payload.size() + sizeof(hash) + sizeof(timestamp_us));

            // Create the nbs packet
            std::vector<uint8_t> nbs({0xE2, 0x98, 0xA2});

            // Size
            for (size_t i = 0; i < sizeof(size); ++i) {
                nbs.push_back(uint8_t((size >> (i * 8)) & 0xFF));
            }
            // Timestamp
            for (size_t i = 0; i < sizeof(timestamp_us); ++i) {
                nbs.push_back(uint8_t((timestamp_us >> (i * 8)) & 0xFF));
            }
            // Hash
            for (size_t i = 0; i < sizeof(hash); ++i) {
                nbs.push_back(uint8_t((hash >> (i * 8)) & 0xFF));
            }
            // Payload
            nbs.insert(nbs.end(), payload.begin(), payload.end());

            // Send the packet to the device
            emit(std::make_unique<TransmitData>(nbs));
        }
    };
}  // namespace module::platform::NUSense

#endif  // MODULE_PLATFORM_NUSENSE_HARDWAREIO_HPP
