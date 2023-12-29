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

#ifndef MODULES_INPUT_NATNET_PARSE_HPP
#define MODULES_INPUT_NATNET_PARSE_HPP

#include <nuclear>

#include "NatNet.hpp"

#include "message/input/MotionCapture.hpp"

namespace module::input {

    using Marker        = message::input::MotionCapture::Marker;
    using MarkerSet     = message::input::MotionCapture::MarkerSet;
    using RigidBody     = message::input::MotionCapture::RigidBody;
    using Skeleton      = message::input::MotionCapture::Skeleton;
    using LabeledMarker = message::input::MotionCapture::LabeledMarker;
    using ForcePlate    = message::input::MotionCapture::ForcePlate;
    using Device        = message::input::MotionCapture::Device;

    // Read plain old datatypes
    template <typename T>
    struct ReadData {
        static inline const typename std::enable_if<std::is_trivial<T>::value, T>::type& read(
            const char*& ptr,
            const uint32_t /*version*/) {

            const T& val = *reinterpret_cast<const T*>(ptr);
            ptr += sizeof(T);
            return val;
        }
    };

    // Read std::strings
    template <>
    struct ReadData<std::string> {
        static inline std::string read(const char*& ptr, const uint32_t /*version*/) {

            std::string str(ptr);
            ptr += str.size() + 1;
            return str;
        }
    };

    // Read Eigen vectors
    template <typename Scalar, int rows, int cols>
    struct ReadData<Eigen::Matrix<Scalar, rows, cols>> {
        static inline Eigen::Matrix<Scalar, rows, cols> read(const char*& ptr, const uint32_t /*version*/) {

            auto data =
                Eigen::Map<const Eigen::Matrix<Scalar, rows, cols>>(reinterpret_cast<const Scalar*>(ptr), rows, cols);
            ptr += sizeof(Scalar) * rows * cols;
            return data;
        }
    };

    // Read std::vectors of things
    template <typename T>
    struct ReadData<std::vector<T>> {
        static inline std::vector<T> read(const char*& ptr, const uint32_t version) {

            // Make a vector of the correct length
            std::vector<T> data(ReadData<uint32_t>::read(ptr, version));

            // Read the data elements
            for (T& d : data) {
                d = ReadData<T>::read(ptr, version);
            }
            return data;
        }
    };

    // Read Markers (they must be read as a vector as they are split into parallel arrays)
    template <>
    struct ReadData<std::vector<Marker>> {
        static inline std::vector<Marker> read(const char*& ptr, const uint32_t version) {

            // Create a vector of the correct length to hold the markers
            std::vector<Marker> markers(ReadData<uint32_t>::read(ptr, version));

            // Read all the positions
            for (auto& marker : markers) {
                marker.position = ReadData<Eigen::Matrix<float, 3, 1>>::read(ptr, version);
            }

            // If we are version 2 or greater we have additional information
            if (version >= 0x02000000) {
                for (auto& marker : markers) {
                    marker.id = ReadData<uint32_t>::read(ptr, version);
                }
                for (auto& marker : markers) {
                    marker.size = ReadData<float>::read(ptr, version);
                }
            }
            else {
                for (auto& marker : markers) {
                    marker.id   = -1;
                    marker.size = -1;
                }
            }

            return markers;
        }
    };

    // Read Marker Sets
    template <>
    struct ReadData<MarkerSet> {
        static inline MarkerSet read(const char*& ptr, const uint32_t version) {

            MarkerSet set;

            set.name               = ReadData<std::string>::read(ptr, version);
            auto markers_positions = ReadData<std::vector<Eigen::Matrix<float, 3, 1>>>::read(ptr, version);
            set.markers.reserve(markers_positions.size());

            // Build markers
            for (const auto& position : markers_positions) {
                Marker marker;
                marker.position = position;
                marker.id       = -1;
                marker.size     = -1;
                set.markers.push_back(marker);
            }

            return set;
        }
    };

    // Read Rigid Bodies
    template <>
    struct ReadData<RigidBody> {
        static inline RigidBody read(const char*& ptr, const uint32_t version) {

            RigidBody rigid_body;

            rigid_body.id       = ReadData<uint32_t>::read(ptr, version);
            rigid_body.position = ReadData<Eigen::Matrix<float, 3, 1>>::read(ptr, version);
            rigid_body.rotation = ReadData<Eigen::Matrix<float, 4, 1>>::read(ptr, version);

            // Version specific information
            if (version < 0x03000000) {
                rigid_body.markers = ReadData<std::vector<Marker>>::read(ptr, version);
            }
            rigid_body.error = version >= 0x02000000 ? ReadData<float>::read(ptr, version) : -1;
            rigid_body.tracking_valid =
                version >= 0x02060000 ? (ReadData<short>::read(ptr, version) & 0x01) == 0x01 : true;

            return rigid_body;
        }
    };

    // Read Skeletons
    template <>
    struct ReadData<Skeleton> {
        static inline Skeleton read(const char*& ptr, const uint32_t version) {

            Skeleton skeleton;

            skeleton.id    = ReadData<uint32_t>::read(ptr, version);
            skeleton.bones = ReadData<std::vector<RigidBody>>::read(ptr, version);

            return skeleton;
        }
    };

    // Read Labeled Markers
    template <>
    struct ReadData<LabeledMarker> {
        static inline LabeledMarker read(const char*& ptr, const uint32_t version) {

            LabeledMarker marker;

            marker.marker.id       = ReadData<uint32_t>::read(ptr, version);
            marker.marker.position = ReadData<Eigen::Matrix<float, 3, 1>>::read(ptr, version);
            marker.marker.size     = ReadData<float>::read(ptr, version);

            if (version >= 0x02060000) {
                short params              = ReadData<short>::read(ptr, version);
                marker.occluded           = (params & 0x01) == 0x01;
                marker.point_cloud_solved = (params & 0x02) == 0x02;
                marker.model_solved       = (params & 0x04) == 0x04;
                if (version >= 0x03000000) {
                    marker.has_model     = (params & 0x08) == 0x08;
                    marker.unlabeled     = (params & 0x10) == 0x10;
                    marker.active_marker = (params & 0x20) == 0x20;
                    marker.err           = ReadData<uint32_t>::read(ptr, version);
                }
            }
            else {
                marker.occluded           = false;
                marker.point_cloud_solved = false;
                marker.model_solved       = false;
            }

            return marker;
        }
    };

    // Read Force Plates
    template <>
    struct ReadData<ForcePlate> {
        static inline ForcePlate read(const char*& ptr, const uint32_t version) {

            ForcePlate force_plate;

            force_plate.id = ReadData<uint32_t>::read(ptr, version);
            auto channels  = ReadData<std::vector<std::vector<float>>>::read(ptr, version);
            force_plate.channels.reserve(channels.size());
            for (uint channel = 0; channel < channels.size(); channel++) {
                force_plate.channels[channel].channel = std::move(channels[channel]);
            }

            return force_plate;
        }
    };

    // Read Device Models
    template <>
    struct ReadData<Device> {
        static inline Device read(const char*& ptr, const uint32_t version) {
            Device device;

            device.id     = ReadData<uint32_t>::read(ptr, version);
            auto channels = ReadData<std::vector<std::vector<float>>>::read(ptr, version);
            device.channels.reserve(channels.size());
            for (uint channel = 0; channel < channels.size(); channel++) {
                device.channels[channel].channel = std::move(channels[channel]);
            }

            return device;
        }
    };

    // Read Marker Set Models
    template <>
    struct ReadData<NatNet::MarkerSetModel> {
        static inline NatNet::MarkerSetModel read(const char*& ptr, const uint32_t version) {

            NatNet::MarkerSetModel m;
            m.name         = ReadData<std::string>::read(ptr, version);
            m.marker_names = ReadData<std::vector<std::string>>::read(ptr, version);
            return m;
        }
    };

    // Read Rigid Body Models
    template <>
    struct ReadData<NatNet::RigidBodyModel> {
        static inline NatNet::RigidBodyModel read(const char*& ptr, const uint32_t version) {

            NatNet::RigidBodyModel m;
            m.name      = version >= 0x02000000 ? ReadData<std::string>::read(ptr, version) : "";
            m.id        = ReadData<uint32_t>::read(ptr, version);
            m.parent_id = ReadData<uint32_t>::read(ptr, version);
            m.offset    = ReadData<Eigen::Matrix<float, 3, 1>>::read(ptr, version);

            uint32_t n_markers = ReadData<uint32_t>::read(ptr, version);
            const char* ptr2   = ptr + (n_markers * 3 * sizeof(float));
            const char* ptr3   = ptr2 + (n_markers * (sizeof(int)));

            if (version >= 0x03000000) {

                std::vector<NatNet::RigidBodyMarker> data(n_markers);

                for (NatNet::RigidBodyMarker& model : data) {
                    // Marker position
                    model.position = ReadData<Eigen::Matrix<float, 3, 1>>::read(ptr, version);
                    // Marker Required Active Labels
                    model.active_label = ReadData<uint32_t>::read(ptr2, version);

                    if (version >= 0x04000000) {
                        // Marker Name
                        model.marker_name = ReadData<std::string>::read(ptr3, version);
                    }
                    m.rigid_body_markers[model.active_label] = model;
                }
            }
            ptr = ptr3;
            return m;
        }
    };

    // Read Skeleton Models
    template <>
    struct ReadData<NatNet::SkeletonModel> {
        static inline NatNet::SkeletonModel read(const char*& ptr, const uint32_t version) {

            NatNet::SkeletonModel m;
            m.name = ReadData<std::string>::read(ptr, version);
            m.id   = ReadData<uint32_t>::read(ptr, version);

            // Convert our bone models into a map
            auto bone_models = ReadData<std::vector<NatNet::RigidBodyModel>>::read(ptr, version);
            for (auto& model : bone_models) {
                m.bone_models[model.id] = model;
            }

            return m;
        }
    };

    // Read ForcePlate Models
    template <>
    struct ReadData<NatNet::ForcePlateModel> {
        static inline NatNet::ForcePlateModel read(const char*& ptr, const uint32_t version) {

            NatNet::ForcePlateModel m;
            if (version >= 0x03000000) {
                m.id                 = ReadData<uint32_t>::read(ptr, version);
                m.name               = ReadData<std::string>::read(ptr, version);
                m.width              = ReadData<uint32_t>::read(ptr, version);
                m.length             = ReadData<uint32_t>::read(ptr, version);
                m.origin             = ReadData<Eigen::Matrix<float, 3, 1>>::read(ptr, version);
                m.calibration_matrix = ReadData<Eigen::Matrix<uint32_t, 12, 12>>::read(ptr, version);
                m.corners            = ReadData<Eigen::Matrix<uint32_t, 4, 3>>::read(ptr, version);
                m.plate_type         = ReadData<uint32_t>::read(ptr, version);
                m.channel_type       = ReadData<uint32_t>::read(ptr, version);
                m.channels           = ReadData<uint32_t>::read(ptr, version);
                for (uint32_t n = 1; n < m.channels; n++) {
                    ReadData<std::string>::read(ptr, version);
                }
            }

            return m;
        }
    };

    // Read Device Models
    template <>
    struct ReadData<NatNet::DeviceModel> {
        static inline NatNet::DeviceModel read(const char*& ptr, const uint32_t version) {

            NatNet::DeviceModel m;
            if (version >= 0x03000000) {
                m.id                = ReadData<uint32_t>::read(ptr, version);
                m.name              = ReadData<std::string>::read(ptr, version);
                m.serial_no         = ReadData<std::string>::read(ptr, version);
                m.device_type       = ReadData<uint32_t>::read(ptr, version);
                m.channel_data_type = ReadData<uint32_t>::read(ptr, version);
                m.channels          = ReadData<uint32_t>::read(ptr, version);
                for (uint32_t n = 1; n < m.channels; n++) {
                    ReadData<std::string>::read(ptr, version);
                }
            }

            return m;
        }
    };

    // Read Camera Models
    template <>
    struct ReadData<NatNet::CameraModel> {
        static inline NatNet::CameraModel read(const char*& ptr, const uint32_t version) {

            NatNet::CameraModel m;
            if (version >= 0x03000000) {
                m.name        = ReadData<std::string>::read(ptr, version);
                m.position    = ReadData<Eigen::Matrix<float, 3, 1>>::read(ptr, version);
                m.orientation = ReadData<Eigen::Matrix<float, 4, 1>>::read(ptr, version);
            }

            return m;
        }
    };
}  // namespace module::input

#endif  // MODULES_INPUT_NATNET_PARSE_HPP
