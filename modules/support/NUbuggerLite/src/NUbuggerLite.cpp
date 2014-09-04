/*
 * This file is part of NUbots Codebase.
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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "NUbuggerLite.h"
#include "messages/support/Configuration.h"
#include "messages/support/GlobalConfig.h"
#include "messages/support/nubugger/proto/Message.pb.h"

#include "utility/time/time.h"

extern "C" {
    #include <sys/socket.h>
    #include <arpa/inet.h>
}

namespace modules {
namespace support {
    using messages::support::nubugger::proto::MessageLite;
    using utility::time::getUtcTimestamp;

    using messages::support::Configuration;
    using messages::support::GlobalConfig;

    NUbuggerLite::NUbuggerLite(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), socket(0) {

        auto setup = [this] (const Configuration<NUbuggerLite>& config, const GlobalConfig& globalConfig) {

            // TODO use an eventfd to allow changing the port dynamically

            PLAYER_ID = globalConfig.playerId;
            TEAM_ID = globalConfig.teamId;
            CLIENT_IP = config["clientIP"].as<std::string>();

            port = config["port"].as<uint>();
            sockaddr_in socketAddress;
            memset(&socketAddress, 0, sizeof(socketAddress));
            socketAddress.sin_family = AF_INET;
            socketAddress.sin_port = htons(port);
            socketAddress.sin_addr.s_addr = htonl(INADDR_ANY);

            int newSocket = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (::bind(newSocket, reinterpret_cast<sockaddr*>(&socketAddress), sizeof(sockaddr)) < 0) {
                throw std::system_error(errno, std::system_category());
            }

            int oldSocket = socket.exchange(newSocket);

            if (oldSocket) {
                ::close(oldSocket);
            }

        };

        on<With<Configuration<NUbuggerLite>>, Trigger<GlobalConfig>>("NUbuggerLite Configuration", setup);
        on<Trigger<Configuration<NUbuggerLite>>, With<GlobalConfig>>("NUbuggerLite Configuration", setup);

        on<Trigger<Every<1, std::chrono::seconds>>, Options<Single, Priority<NUClear::LOW>>>([this] (const time_t&) {
            if (socket) {
                sockaddr_in socketAddress;
                memset(&socketAddress, 0, sizeof(socketAddress));
                socketAddress.sin_family = AF_INET;
                socketAddress.sin_port = htons(port);
                socketAddress.sin_addr.s_addr = inet_addr(CLIENT_IP.c_str());

                MessageLite message;
                message.set_utc_timestamp(getUtcTimestamp());
                message.set_test("lite test");

                size_t messageSize = message.ByteSize();
                std::string packet = message.SerializeAsString();

                if (::sendto(socket, packet.data(), messageSize, 0, reinterpret_cast<sockaddr*>(&socketAddress), sizeof(socketAddress)) < 0) {
                    throw std::system_error(errno, std::system_category());
                }
            }
        });
    }

}
}

