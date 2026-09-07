/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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
#ifndef UTILITY_PLATFORM_BOOSTER_CHANNEL_FACTORY_HPP
#define UTILITY_PLATFORM_BOOSTER_CHANNEL_FACTORY_HPP

#include <booster/robot/channel/channel_factory.hpp>
#include <mutex>
#include <string>

namespace utility::platform::Booster {

    /**
     * @brief Idempotently initialise the process-wide Booster DDS ChannelFactory.
     *
     * ChannelFactory is a singleton shared by every module that talks to the robot over DDS
     * (K1Camera, platform::Booster::HardwareIO, ...) and must only be initialised once per process.
     * NUClear does not order on<Startup> reactions between modules, so no module can assume another
     * has already initialised it. Every module that needs DDS should call this instead of calling
     * ChannelFactory::Instance()->Init() directly; the first caller wins and the rest are no-ops.
     *
     * @param domain_id         DDS domain to join. BoosterOS publishes on domain 0.
     * @param network_interface Interface to bind to, or empty to let the SDK choose.
     */
    inline void ensure_channel_factory(const int32_t domain_id = 0, const std::string& network_interface = "") {
        static std::once_flag flag;
        std::call_once(flag, [&] {
            ::booster::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);
        });
    }

}  // namespace utility::platform::Booster

#endif  // UTILITY_PLATFORM_BOOSTER_CHANNEL_FACTORY_HPP
