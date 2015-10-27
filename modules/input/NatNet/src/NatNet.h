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
 * Copyright 2015 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_INPUT_NATNET_H
#define MODULES_INPUT_NATNET_H

#include <nuclear>

namespace modules {
namespace input {

    class NatNet : public NUClear::Reactor {
    private:
        struct Packet {
            enum class Type : uint16_t {
                PING                  = 0,
                PING_RESPONSE         = 1,
                REQUEST               = 2,
                RESPONSE              = 3,
                REQUEST_MODELDEF      = 4,
                MODEL_DEF             = 5,
                REQUEST_FRAMEOFDATA   = 6,
                FRAME_OF_DATA         = 7,
                MESSAGE_STRING        = 8,
                UNRECOGNIZED_REQUEST  = 100
            };

            Type type;
            uint16_t length;
            char data;
        };

        // The version of NatNet we are running with
        uint32_t remote = 0;
        uint32_t version = 0;

        uint16_t commandPort = 0;
        uint16_t dataPort = 0;
        std::string multicastAddress = "";
        ReactionHandle commandHandle;
        ReactionHandle dataHandle;
        int commandFd = 0;

        void processFrame(const Packet& packet);
        void sendCommand(Packet::Type type, std::vector<char> data = std::vector<char>());
        void processModel(const Packet& packet);
        void processPing(const Packet& packet);
        void processResponse(const Packet& packet);
        void processString(const Packet& packet);
        void process(const std::vector<char>& input);

    public:
        /// @brief Called by the powerplant to build and setup the NatNet reactor.
        explicit NatNet(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULES_INPUT_NATNET_H
