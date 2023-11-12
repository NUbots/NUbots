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

#include "NatNet.hpp"

#include <fmt/format.h>

#include "Parse.hpp"

#include "extension/Configuration.hpp"

namespace module::input {

    using extension::Configuration;

    using message::input::MotionCapture;

    NatNet::NatNet(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("NatNet.yaml").then([this](const Configuration& config) {
            // We are updating to a new multicast address
            if (cfg.multicast_address != config["multicast_address"].as<std::string>()
                || cfg.data_port != config["data_port"].as<uint32_t>()
                || cfg.command_port != config["command_port"].as<uint32_t>()) {

                if (command_handle) {
                    command_handle.unbind();
                }
                if (data_handle) {
                    data_handle.unbind();
                }

                // TODO: Convert this so that it uses the cfg struct
                // Set our new variables
                cfg.multicast_address = config["multicast_address"].as<std::string>();
                cfg.data_port         = config["data_port"].as<uint16_t>();
                cfg.command_port      = config["command_port"].as<uint16_t>();

                log<NUClear::INFO>("Connecting to NatNet network", cfg.multicast_address);

                // Create a listening UDP port for commands
                std::tie(command_handle, std::ignore, commandFd) =
                    on<UDP>().then("NatNet Command", [this](const UDP::Packet& packet) { process(packet.payload); });

                // Create a listening UDP port for data
                std::tie(data_handle, std::ignore, std::ignore) =
                    on<UDP::Multicast>(cfg.multicast_address, cfg.data_port)
                        .then("NatNet Data", [this](const UDP::Packet& packet) {
                            // Test if we are "connected" to this remote
                            // And if we are we can use the data
                            auto address = static_cast<uint32_t>(std::stoul(packet.remote.address));
                            if (remote == address && version != 0) {
                                process(packet.payload);
                            }
                            // We have started connecting but haven't received a return ping
                            else if (remote == address && version == 0) {
                                // TODO(HardwareTeam): maybe set a timeout here to try again
                            }
                            // We haven't connected to anything yet
                            else if (remote == 0) {
                                // This is now our remote
                                remote = address;

                                // Send a ping command
                                send_command(Packet::Type::PING);
                            }
                            else if (remote != address) {
                                log<NUClear::WARN>("There is more than one NatNet server running on this network");
                            }
                        });
            }
            cfg.dump_packets = config["dump_packets"].as<bool>();
        });
    }

    void NatNet::send_command(Packet::Type type, std::vector<char> data) {
        if (remote > 0) {
            // Make a vector to hold our packet
            std::vector<char> packet(sizeof(Packet) - 1);

            // Fill in the header
            auto* header   = reinterpret_cast<Packet*>(packet.data());
            header->type   = type;
            header->length = data.size();

            // Fill in the data
            packet.insert(packet.end(), data.begin(), data.end());

            // Work out our remotes address
            sockaddr_in address{};
            memset(&address, 0, sizeof(sockaddr_in));
            address.sin_family      = AF_INET;
            address.sin_port        = htons(cfg.command_port);
            address.sin_addr.s_addr = htonl(remote);

            // Send to our remote server
            ::sendto(commandFd,
                     packet.data(),
                     packet.size(),
                     0,
                     reinterpret_cast<sockaddr*>(&address),
                     sizeof(sockaddr));
        }
        else {
            log<NUClear::WARN>("NatNet is not yet connected to a remote server");
        }
    }

    void NatNet::process_frame(const Packet& packet) {

        // Our output motion capture object
        auto mocap = std::make_unique<MotionCapture>();

        // Our pointer as we move through the data
        const char* ptr = &packet.data;

        // Read frame number
        mocap->frame_number = ReadData<uint32_t>::read(ptr, version);

        // Read the MarkerSets
        mocap->marker_sets = ReadData<std::vector<MotionCapture::MarkerSet>>::read(ptr, version);

        // Read the free floating markers
        auto free_markers = ReadData<std::vector<Eigen::Vector3f>>::read(ptr, version);
        mocap->markers.reserve(free_markers.size());
        // Build markers
        for (const auto& position : free_markers) {
            MotionCapture::Marker marker;
            marker.position = position;
            marker.id       = -1;
            marker.size     = -1;
            mocap->markers.push_back(marker);
        }

        // Read the Rigid Bodies
        mocap->rigid_bodies = ReadData<std::vector<MotionCapture::RigidBody>>::read(ptr, version);

        // Read the skeletons
        if (version >= 0x02010000) {
            mocap->skeletons = ReadData<std::vector<MotionCapture::Skeleton>>::read(ptr, version);
        }

        // Read the labeled markers
        if (version >= 0x02030000) {
            mocap->labeled_markers = ReadData<std::vector<MotionCapture::LabeledMarker>>::read(ptr, version);
        }

        // Read the force plates
        if (version >= 0x02090000) {
            mocap->force_plates = ReadData<std::vector<MotionCapture::ForcePlate>>::read(ptr, version);
        }

        // Read the Devices
        if (version >= 0x03000000) {
            mocap->devices = ReadData<std::vector<MotionCapture::Device>>::read(ptr, version);
        }

        // Read our metadata
        if (version < 0x03000000) {
            mocap->latency = ReadData<float>::read(ptr, version);
        }
        mocap->timecode     = ReadData<uint32_t>::read(ptr, version);
        mocap->timecode_sub = ReadData<uint32_t>::read(ptr, version);

        // In version 2.7 natnet_timestamp changed from a float to a double
        if (version >= 0x02070000) {
            mocap->natnet_timestamp = ReadData<double>::read(ptr, version);
        }
        else {
            mocap->natnet_timestamp = ReadData<float>::read(ptr, version);
        }

        // High res timestamps
        if (version >= 0x03000000) {
            mocap->mid_exposure_timestamp  = ReadData<double>::read(ptr, version);
            mocap->data_received_timestamp = ReadData<double>::read(ptr, version);
            mocap->transmit_timestamp      = ReadData<double>::read(ptr, version);
        }

        short params                  = ReadData<short>::read(ptr, version);
        mocap->recording              = (params & 0x01) == 0x01;
        mocap->tracked_models_changed = (params & 0x01) == 0x02;

        // TODO(HardwareTeam): there is an eod thing here
        uint32_t eod = ReadData<uint32_t>::read(ptr, version);
        if (eod != 0) {
            log<NUClear::ERROR>("Packet not read correctly, Abandoning.");
            return;
        }

        // Apply the model information we have to the objects
        for (auto& marker_set : mocap->marker_sets) {

            auto model = marker_set_models.find(marker_set.name);

            // We have a model
            if (model != marker_set_models.end()) {
            }
            // We need to update our models
            else {
                // Inform that we are updating our models
                log<NUClear::INFO>("NatNet models are out of date, updating before resuming data");

                // Request model definitions
                send_command(Packet::Type::REQUEST_MODEL_DEFINITIONS);

                // Stop processing
                return;
            }
        }

        for (auto& rigid_body : mocap->rigid_bodies) {

            auto model = rigid_body_models.find(rigid_body.id);

            // We have a model
            if (model != rigid_body_models.end()) {

                rigid_body.name   = model->second.name;
                rigid_body.offset = model->second.offset;

                auto parent =
                    std::find_if(mocap->rigid_bodies.begin(),
                                 mocap->rigid_bodies.end(),
                                 [model](const MotionCapture::RigidBody& rb) { return rb.id == model->second.id; });

                // Get a pointer to our parent if it exists and is not us
                rigid_body.parent = parent->id == rigid_body.id ? 0
                                    : parent == mocap->rigid_bodies.end()
                                        ? -1
                                        : std::distance(mocap->rigid_bodies.begin(), parent);
            }
            // We need to update our models
            else {
                // Inform that we are updating our models
                log<NUClear::INFO>("NatNet models are out of date, updating before resuming data");

                // Request model definitions
                send_command(Packet::Type::REQUEST_MODEL_DEFINITIONS);

                // Stop processing
                return;
            }
        }

        // Now we reverse link all our rigid bodies
        for (auto rigid_body = mocap->rigid_bodies.begin(); rigid_body != mocap->rigid_bodies.end(); rigid_body++) {
            if (rigid_body->parent > 0) {
                mocap->rigid_bodies.at(rigid_body->parent)
                    .children.push_back(std::distance(mocap->rigid_bodies.begin(), rigid_body));
            }
        }

        for (auto& skeleton : mocap->skeletons) {

            auto model = skeleton_models.find(skeleton.id);

            // We have a model
            if (model != skeleton_models.end()) {

                for (auto& bone : skeleton.bones) {
                    auto bone_model = model->second.bone_models.find(bone.id);
                    // We have a model for this bone
                    if (bone_model != model->second.bone_models.end()) {

                        bone.name   = bone_model->second.name;
                        bone.offset = bone_model->second.offset;

                        auto parent = std::find_if(skeleton.bones.begin(),
                                                   skeleton.bones.end(),
                                                   [bone_model](const MotionCapture::RigidBody& rb) {
                                                       return rb.id == bone_model->second.id;
                                                   });

                        bone.parent = parent->id == bone.id            ? 0
                                      : parent == skeleton.bones.end() ? -1
                                                                       : std::distance(skeleton.bones.begin(), parent);
                    }
                    // We need to update our models
                    else {
                        // Inform that we are updating our models
                        log<NUClear::INFO>("NatNet models are out of date, updating before resuming data");

                        // Request model definitions
                        send_command(Packet::Type::REQUEST_MODEL_DEFINITIONS);

                        // Stop processing
                        return;
                    }
                }
            }
            // We need to update our models
            else {
                // Inform that we are updating our models
                log<NUClear::INFO>("NatNet models are out of date, updating before resuming data");

                // Request model definitions
                send_command(Packet::Type::REQUEST_MODEL_DEFINITIONS);

                // Stop processing
                return;
            }

            // Now we reverse link all our bones
            for (auto rigid_body = skeleton.bones.begin(); rigid_body != skeleton.bones.end(); rigid_body++) {
                if (rigid_body->parent > 0) {
                    skeleton.bones.at(rigid_body->parent)
                        .children.push_back(std::distance(skeleton.bones.begin(), rigid_body));
                }
            }
        }

        // Emit our frame
        emit(std::move(mocap));
    }

    void NatNet::process_model(const Packet& packet) {

        log<NUClear::INFO>("Updating model definitions");

        // Our pointer as we move through the data
        const char* ptr = &packet.data;

        uint32_t n_models = ReadData<uint32_t>::read(ptr, version);

        for (uint32_t i = 0; i < n_models; ++i) {
            // Read the type
            uint32_t type = ReadData<uint32_t>::read(ptr, version);

            // Parse the correct type
            switch (type) {
                // Marker Set
                case 0: {
                    MarkerSetModel m          = ReadData<MarkerSetModel>::read(ptr, version);
                    marker_set_models[m.name] = m;
                } break;

                // Rigid Body
                case 1: {
                    RigidBodyModel m        = ReadData<RigidBodyModel>::read(ptr, version);
                    rigid_body_models[m.id] = m;
                } break;

                // Skeleton
                case 2: {
                    SkeletonModel m       = ReadData<SkeletonModel>::read(ptr, version);
                    skeleton_models[m.id] = m;
                } break;

                // Force Plate
                case 3: {
                    ForcePlateModel m        = ReadData<ForcePlateModel>::read(ptr, version);
                    force_plate_models[m.id] = m;
                } break;

                // Device
                case 4: {
                    DeviceModel m       = ReadData<DeviceModel>::read(ptr, version);
                    device_models[m.id] = m;
                } break;

                // Camera
                case 5: {
                    static uint32_t n = 0;
                    CameraModel m     = ReadData<CameraModel>::read(ptr, version);
                    camera_models[n]  = m;
                    n++;
                } break;

                // Bad packet
                default: {
                    log<NUClear::WARN>("NatNet received an unexpected model type", type);
                } break;
            }
        }
    }

    void NatNet::process_ping(const Packet& packet) {

        // Extract the information from the packet
        std::string name(&packet.data);
        const char* app_version     = &packet.data + 256;
        const char* nat_net_version = app_version + 4;

        // Update our version number
        version = ntohl(*reinterpret_cast<const uint32_t*>(nat_net_version));
        // Make our app version a string (removing trailing 0 version numbers)
        std::string str_app_version = std::to_string(int(app_version[0]))
                                      + (app_version[1] == 0 ? "" : "." + std::to_string(app_version[1]))
                                      + (app_version[2] == 0 ? "" : "." + std::to_string(app_version[2]))
                                      + (app_version[3] == 0 ? "" : "." + std::to_string(app_version[3]));

        // Make our nat_net_version a string
        std::string str_nat_version = std::to_string(nat_net_version[0])
                                      + (nat_net_version[1] == 0 ? "" : "." + std::to_string(nat_net_version[1]))
                                      + (nat_net_version[2] == 0 ? "" : "." + std::to_string(nat_net_version[2]))
                                      + (nat_net_version[3] == 0 ? "" : "." + std::to_string(nat_net_version[3]));

        // Make our remote into an IP
        std::array<char, INET_ADDRSTRLEN> str{};
        inet_ntop(AF_INET, &remote, str.data(), str.size());
        std::string str_remote = str.data();

        log<NUClear::INFO>(
            fmt::format("Connected to {} ({} {}) over NatNet {}", str_remote, name, str_app_version, str_nat_version));

        // Request model definitions on startup
        send_command(Packet::Type::REQUEST_MODEL_DEFINITIONS);
    }

    void NatNet::process_response(const Packet& /*packet*/) {
        // if(gCommandResponseSize==4)
        //     memcpy(&gCommandResponse, &PacketIn.Data.lData[0], gCommandResponseSize);
        // else
        // {
        //     memcpy(&gCommandResponseString[0], &PacketIn.Data.cData[0], gCommandResponseSize);
        //     printf("Response : %s", gCommandResponseString);
        //     gCommandResponse = 0;   // ok
        // }
    }

    void NatNet::process_string(const Packet& packet) {
        std::string str(&packet.data, packet.length);

        // TODO(HardwareTeam): do something with this string?
    }


    void NatNet::process(const std::vector<uint8_t>& input) {
        // Get our packet
        const Packet& packet = *reinterpret_cast<const Packet*>(input.data());

        if (cfg.dump_packets) {
            static int i = 0;
            std::vector<char> input_char(input.begin(), input.end());
            std::ofstream output(fmt::format("natnet_{:06d}.bin", i++));
            output.write(input_char.data(), input_char.size());
        }

        // Work out it's type
        switch (packet.type) {
            case Packet::Type::PING_RESPONSE: process_ping(packet); break;

            case Packet::Type::RESPONSE: process_response(packet); break;

            case Packet::Type::MODEL_DEF: process_model(packet); break;

            case Packet::Type::FRAME_OF_DATA: process_frame(packet); break;

            case Packet::Type::UNRECOGNIZED_REQUEST:
                log<NUClear::ERROR>("An unrecognized request was made to the NatNet server");
                break;

            case Packet::Type::MESSAGE_STRING: process_string(packet); break;

            default: log<NUClear::ERROR>("The NatNet server sent an unexpected packet type"); break;
        }
    }
}  // namespace module::input
