/*
 * MIT License
 *
 * Copyright (c) 2015 NUbots
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

#ifndef MODULES_INPUT_NATNET_HPP
#define MODULES_INPUT_NATNET_HPP

#include <Eigen/Core>
#include <nuclear>

namespace module::input {

    class NatNet : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            Config() = default;
            /// @brief Motive Multicast Address
            std::string multicast_address = "";
            /// @brief UDP Command port for communicating with motive
            uint32_t command_port = 0;
            /// @brief UDP Data port for communicating with motive
            uint32_t data_port = 0;
            /// @brief Allows motive packets to be dumped to file
            bool dump_packets = false;
        } cfg;

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

            Packet() = default;
            Type type{Type::PING};
            uint16_t length{0};
            char data{0};
        };

        struct MarkerSetModel {
            MarkerSetModel() = default;
            std::string name{};
            std::vector<std::string> marker_names;
        };

        struct RigidBodyMarker {
            Eigen::Vector3f position = Eigen::Vector3f::Zero();
            uint32_t active_label;
            std::string marker_name;
        };

        struct RigidBodyModel {
            RigidBodyModel() = default;
            std::string name{};
            uint32_t id{0};
            uint32_t parent_id{0};
            Eigen::Vector3f offset   = Eigen::Vector3f::Zero();
            Eigen::Vector3f rotation = Eigen::Vector3f::Zero();
            std::map<uint32_t, RigidBodyMarker> rigid_body_markers;
        };

        struct SkeletonModel {
            SkeletonModel() = default;
            std::string name{};
            uint32_t id{0};
            std::map<uint32_t, RigidBodyModel> bone_models;
        };

        struct ForcePlateModel {
            ForcePlateModel() = default;
            std::string name{};
            uint32_t id{0};
            uint32_t width{0};
            uint32_t length{0};
            Eigen::Vector3f origin = Eigen::Vector3f::Zero();
            Eigen::Matrix<uint32_t, 12, 12> calibration_matrix;
            Eigen::Matrix<uint32_t, 4, 3> corners;
            uint32_t plate_type{0};
            uint32_t channel_type{0};
            uint32_t channels{0};
        };

        struct DeviceModel {
            DeviceModel() = default;
            std::string name{};
            uint32_t id{0};
            std::string serial_no{};
            uint32_t device_type{0};
            uint32_t channel_data_type{0};
            uint32_t channels{0};
        };

        struct CameraModel {
            CameraModel() = default;
            std::string name;
            Eigen::Vector3f position    = Eigen::Vector3f::Zero();
            Eigen::Vector4f orientation = Eigen::Vector4f::Zero();
        };

        // Models we are using
        std::map<std::string, MarkerSetModel> marker_set_models = {};
        std::map<uint32_t, RigidBodyModel> rigid_body_models    = {};
        std::map<uint32_t, SkeletonModel> skeleton_models       = {};
        std::map<uint32_t, ForcePlateModel> force_plate_models  = {};
        std::map<uint32_t, DeviceModel> device_models           = {};
        /// @brief Camera model number in motive is not associated with map key number
        std::map<uint32_t, CameraModel> camera_models = {};

        // The version of NatNet we are running with
        uint32_t remote  = 0;
        uint32_t version = 0;

        ReactionHandle command_handle;
        ReactionHandle data_handle;
        int commandFd = 0;

        void process_frame(const Packet& packet);
        void send_command(Packet::Type type, std::vector<char> data = std::vector<char>());
        void process_model(const Packet& packet);
        void process_ping(const Packet& packet);
        void process_response(const Packet& packet);
        static void process_string(const Packet& packet);
        void process(const std::vector<char>& input);

        /// @brief Called by the powerplant to build and setup the NatNet reactor.
        explicit NatNet(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::input

#endif  // MODULES_INPUT_NATNET_HPP
