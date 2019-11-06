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

#include <Eigen/Core>
#include <nuclear>

namespace module {
namespace input {

    class NatNet : public NUClear::Reactor {
    public:
        struct Packet {
            enum class Type : uint16_t {
                PING                      = 0,
                PING_RESPONSE             = 1,
                REQUEST                   = 2,
                RESPONSE                  = 3,
                REQUEST_MODEL_DEFINITIONS = 4,
                MODEL_DEF                 = 5,
                REQUEST_FRAMEOFDATA       = 6,
                FRAME_OF_DATA             = 7,
                MESSAGE_STRING            = 8,
                UNRECOGNIZED_REQUEST      = 100
            };

            Packet() : type(Type::PING), length(0), data(0) {}
            Type type;
            uint16_t length;
            char data;
        };

        struct MarkerSetModel {
            MarkerSetModel() : name(""), markerNames() {}
            std::string name;
            std::vector<std::string> markerNames;
        };

        struct RigidBodyModel {
            RigidBodyModel() : name(""), id(0), parentId(0), offset(Eigen::Vector3f::Zero()) {}
            std::string name;
            uint32_t id;
            uint32_t parentId;
            Eigen::Vector3f offset;
        };

        struct SkeletonModel {
            SkeletonModel() : name(""), id(0), boneModels() {}
            std::string name;
            uint32_t id;
            std::map<uint32_t, RigidBodyModel> boneModels;
        };

        // Models we are using
        std::map<std::string, MarkerSetModel> markerSetModels;
        std::map<uint32_t, RigidBodyModel> rigidBodyModels;
        std::map<uint32_t, SkeletonModel> skeletonModels;

        // The version of NatNet we are running with
        uint32_t remote  = 0;
        uint32_t version = 0;

        uint16_t commandPort         = 0;
        uint16_t dataPort            = 0;
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

        /// @brief Called by the powerplant to build and setup the NatNet reactor.
        explicit NatNet(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace input
}  // namespace module

#endif  // MODULES_INPUT_NATNET_H
