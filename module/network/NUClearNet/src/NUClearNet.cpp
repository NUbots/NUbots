/*
 * Copyright (C) 2013-2016 Trent Houliston <trent@houliston.me>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "NUClearNet.hpp"

#include "extension/Configuration.hpp"

namespace module::network {

    using extension::Configuration;

    NUClearNet::NUClearNet(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("NUClearNet.yaml").then([this](const Configuration& config) {
            auto net_config              = std::make_unique<NUClear::message::NetworkConfiguration>();
            net_config->name             = config["name"].as<std::string>();
            net_config->announce_address = config["address"].as<std::string>();
            net_config->announce_port    = config["port"].as<uint16_t>();
            emit<Scope::INLINE>(net_config);
        });

        on<Trigger<NUClear::message::NetworkJoin>>().then([this](const NUClear::message::NetworkJoin& event) {
            auto [addr, port] = event.address.address();
            log<INFO>("Connected to", event.name, "on", addr + ":" + std::to_string(port));
        });

        on<Trigger<NUClear::message::NetworkLeave>>().then([this](const NUClear::message::NetworkLeave& event) {
            auto [addr, port] = event.address.address();
            log<INFO>("Disconnected from", event.name, "on", addr + ":" + std::to_string(port));
        });
    }

}  // namespace module::network
