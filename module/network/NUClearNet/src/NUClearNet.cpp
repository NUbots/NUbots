/*
 * MIT License
 *
 * Copyright (c) 2013 NUbots
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

#include "NUClearNet.hpp"

#include "extension/Configuration.hpp"

#include "utility/support/hostname.hpp"

namespace module::network {

    using extension::Configuration;

    NUClearNet::NUClearNet(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("NUClearNet.yaml").then([this](const Configuration& config) {
            log_level                   = config["log_level"].as<NUClear::LogLevel>();
            auto netConfig              = std::make_unique<NUClear::message::NetworkConfiguration>();
            auto name                   = config["name"].as<std::string>();
            netConfig->name             = name.empty() ? utility::support::getHostname() : name;
            netConfig->announce_address = config["address"].as<std::string>();
            netConfig->announce_port    = config["port"].as<uint16_t>();
            emit<Scope::DIRECT>(netConfig);
        });

        on<Trigger<NUClear::message::NetworkJoin>>().then([this](const NUClear::message::NetworkJoin& event) {
            char c[255];
            std::memset(c, 0, sizeof(c));
            std::string addr;
            int port = 0;

            switch (event.address.sock.sa_family) {
                case AF_INET:
                    addr = inet_ntop(event.address.sock.sa_family, &event.address.ipv4.sin_addr, c, sizeof(c));
                    port = ntohs(event.address.ipv4.sin_port);
                    break;

                case AF_INET6:
                    addr = inet_ntop(event.address.sock.sa_family, &event.address.ipv6.sin6_addr, c, sizeof(c));
                    port = ntohs(event.address.ipv6.sin6_port);
                    break;
            }

            log<NUClear::INFO>("Connected to", event.name, "on", addr + ":" + std::to_string(port));
        });

        on<Trigger<NUClear::message::NetworkLeave>>().then([this](const NUClear::message::NetworkLeave& event) {
            char c[255];
            std::memset(c, 0, sizeof(c));
            std::string addr;
            int port = 0;

            switch (event.address.sock.sa_family) {
                case AF_INET:
                    addr = inet_ntop(event.address.sock.sa_family, &event.address.ipv4.sin_addr, c, sizeof(c));
                    port = ntohs(event.address.ipv4.sin_port);
                    break;

                case AF_INET6:
                    addr = inet_ntop(event.address.sock.sa_family, &event.address.ipv6.sin6_addr, c, sizeof(c));
                    port = ntohs(event.address.ipv6.sin6_port);
                    break;
            }

            log<NUClear::INFO>("Disconnected from", event.name, "on", addr + ":" + std::to_string(port));
        });
    }
}  // namespace module::network
