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

#include "NUcap.h"
#include <nuclear>

#include <iostream>
#include <arpa/inet.h>

#include "NatNetLinux/NatNet.h"
#include "NatNetLinux/CommandListener.h"
#include "NatNetLinux/FrameListener.h"

#include "message/support/Configuration.h"
#include "message/input/proto/MotionCapture.pb.h"

namespace module {
namespace support {

    using message::support::Configuration;
    using message::input::proto::MotionCapture;

    NUcap::NUcap(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("NUcap.yaml").then([this] (const Configuration& config) {

            uint32_t serverIP = inet_addr(config["serverIP"].as<std::string>().c_str());
            uint32_t clientIP = inet_addr(config["clientIP"].as<std::string>().c_str());

            NUClear::log("serverIP", serverIP);
            NUClear::log("clientIP", clientIP);


            // Version number of the NatNet protocol, as reported by the server.
            unsigned char natNetMajor;
            unsigned char natNetMinor;

            // Use this socket address to send commands to the server.
            struct sockaddr_in serverCommands = NatNet::createAddress(serverIP, NatNet::COMMAND_PORT);

            // Create sockets
            sdCommand = NatNet::createCommandSocket( clientIP );
            sdData = NatNet::createDataSocket( clientIP );

            // Start the CommandListener in a new thread.
            commandListener = std::make_unique<CommandListener>(sdCommand);
            commandListener->start();

            // Send a ping packet to the server so that it sends us the NatNet version
            // in its response to commandListener.
            NatNetPacket ping = NatNetPacket::pingPacket();
            ping.send(sdCommand, serverCommands);

            // Wait here for ping response to give us the NatNet version.
            commandListener->getNatNetVersion(natNetMajor, natNetMinor);

            // Start up a FrameListener in a new thread.
            frameListener = std::make_unique<FrameListener>(sdData, natNetMajor, natNetMinor);
            frameListener->start();

        });

        on<Every<60, Per<std::chrono::seconds>>>().then([this] {
            bool valid;
            // Try to get a new frame from the listener.
            MocapFrame frame(frameListener->pop(&valid).first);
            // Quit if the listener has no more frames.
            if (!valid) {
                std::cout << " data not valid" << std::endl;
                return;
            }
//            std::cout << frame << std::endl;

            auto moCap = std::make_unique<MotionCapture>();

            for (auto marker : frame.markerSets()) {
                auto* markerSet = moCap->add_marker_sets();

                markerSet->set_name(marker.name());
                for (auto point : marker.markers()) {
                    auto* markerPoint = markerSet->add_points();
                    auto* position = markerPoint->mutable_position();
                    position->set_x(point.x);
                    position->set_y(point.y);
                    position->set_z(point.z);
                }
            }

            for (auto point : frame.unIdMarkers()) {
                auto* markerPoint = moCap->add_unidentified_points();
//                NUClear::log("Point", point.x, point.y, point.z);
                auto* position = markerPoint->mutable_position();
                position->set_x(point.x);
                position->set_y(point.y);
                position->set_z(point.z);
            }

            for (auto fRigidBody : frame.rigidBodies()) {
                auto* rigidBody = moCap->add_rigid_bodies();
                rigidBody->set_identifier(fRigidBody.id());

                auto* position = rigidBody->mutable_position();
                // normalize to robot coordinate system, x foward, y left, z up
                position->set_x(-fRigidBody.location().z);
                position->set_y(-fRigidBody.location().x);
                position->set_z(fRigidBody.location().y);

                // log("Received:", rigidBody->identifier(), position->x(), position->y(), position->z());
//
                auto fRotation = fRigidBody.orientation();

                auto* rotation = rigidBody->mutable_rotation();
                rotation->set_x(fRotation.qw);
                rotation->set_y(-fRotation.qz);
                rotation->set_z(-fRotation.qx);
                rotation->set_t(fRotation.qy);

                for (auto point : fRigidBody.markers()) {
                    auto* markerPoint = rigidBody->add_points();
                    auto* position = markerPoint->mutable_position();
                    position->set_x(point.x);
                    position->set_y(point.y);
                    position->set_z(point.z);
                }
            }

            emit<Scope::NETWORK,Scope::LOCAL>(std::move(moCap));

        });

        on<Shutdown>().then([this] {
            // Wait for threads to finish.
            frameListener->stop();
            commandListener->stop();
            frameListener->join();
            commandListener->join();

            // Epilogue
            close(sdData);
            close(sdCommand);
        });
    }

}
}

