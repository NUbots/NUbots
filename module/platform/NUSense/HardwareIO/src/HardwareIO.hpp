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

#include <nuclear>
#include <vector>

#include "NUgus.hpp"
#include "util/PacketHandler.hpp"

#include "message/platform/RawSensors.hpp"

#include "utility/io/uart.hpp"
#include "utility/platform/RawSensors.hpp"

namespace module::platform::NUSense {

    class HardwareIO : public NUClear::Reactor {

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

        /// @brief The buffer to store the incoming bytes from the NUSense device
        std::array<uint8_t, 512> nusense_usb_bytes{};

        /// @brief Manage desired port for NUSense
        utility::io::uart nusense{};

        /// @brief The handler for the NUSense packet
        PacketHandler nusense_receiver{};

        /// @brief Contains device information specific to the NUgus robot
        NUgus nugus{};
    };

}  // namespace module::platform::NUSense

#endif  // MODULE_PLATFORM_NUSENSE_HARDWAREIO_HPP
