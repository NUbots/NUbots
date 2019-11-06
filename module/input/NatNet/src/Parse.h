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

#ifndef MODULES_INPUT_NATNET_PARSE_H
#define MODULES_INPUT_NATNET_PARSE_H

#include <nuclear>

#include "NatNet.h"
#include "message/input/MotionCapture.h"

namespace module {
namespace input {

    using Marker        = message::input::MotionCapture::Marker;
    using MarkerSet     = message::input::MotionCapture::MarkerSet;
    using RigidBody     = message::input::MotionCapture::RigidBody;
    using Skeleton      = message::input::MotionCapture::Skeleton;
    using LabeledMarker = message::input::MotionCapture::LabeledMarker;
    using ForcePlate    = message::input::MotionCapture::ForcePlate;

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
    struct ReadData<Eigen::Matrix<Scalar, rows, cols, Eigen::DontAlign>> {
        static inline Eigen::Matrix<Scalar, rows, cols, Eigen::DontAlign> read(const char*& ptr,
                                                                               const uint32_t /*version*/) {

            auto data = Eigen::Map<const Eigen::Matrix<Scalar, rows, cols, Eigen::DontAlign>>(
                reinterpret_cast<const Scalar*>(ptr), rows, cols);
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
                marker.position = ReadData<Eigen::Matrix<float, 3, 1, Eigen::DontAlign>>::read(ptr, version);
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

            set.name = ReadData<std::string>::read(ptr, version);
            auto markersPositions =
                ReadData<std::vector<Eigen::Matrix<float, 3, 1, Eigen::DontAlign>>>::read(ptr, version);
            set.markers.reserve(markersPositions.size());

            // Build markers
            for (auto position : markersPositions) {
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

            RigidBody rigidBody;

            rigidBody.id       = ReadData<uint32_t>::read(ptr, version);
            rigidBody.position = ReadData<Eigen::Matrix<float, 3, 1, Eigen::DontAlign>>::read(ptr, version);
            rigidBody.rotation = ReadData<Eigen::Matrix<float, 4, 1, Eigen::DontAlign>>::read(ptr, version);
            rigidBody.markers  = ReadData<std::vector<Marker>>::read(ptr, version);

            // Version specific information
            rigidBody.error = version >= 0x02000000 ? ReadData<float>::read(ptr, version) : -1;
            rigidBody.trackingValid =
                version >= 0x02060000 ? (ReadData<short>::read(ptr, version) & 0x01) == 0x01 : true;

            return rigidBody;
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
            marker.marker.position = ReadData<Eigen::Matrix<float, 3, 1, Eigen::DontAlign>>::read(ptr, version);
            marker.marker.size     = ReadData<float>::read(ptr, version);

            if (version >= 0x02060000) {
                short params            = ReadData<short>::read(ptr, version);
                marker.occluded         = (params & 0x01) == 0x01;
                marker.pointCloudSolved = (params & 0x02) == 0x02;
                marker.modelSolved      = (params & 0x04) == 0x04;
            }
            else {
                marker.occluded         = false;
                marker.pointCloudSolved = false;
                marker.modelSolved      = false;
            }

            return marker;
        }
    };

    // Read Force Plates
    template <>
    struct ReadData<ForcePlate> {
        static inline ForcePlate read(const char*& ptr, const uint32_t version) {

            ForcePlate forcePlate;

            forcePlate.id = ReadData<uint32_t>::read(ptr, version);
            auto channels = ReadData<std::vector<std::vector<float>>>::read(ptr, version);
            forcePlate.channels.reserve(channels.size());
            for (uint channel = 0; channel < channels.size(); channel++) {
                forcePlate.channels[channel].channel = std::move(channels[channel]);
            }

            return forcePlate;
        }
    };

    // Read Marker Set Models
    template <>
    struct ReadData<NatNet::MarkerSetModel> {
        static inline NatNet::MarkerSetModel read(const char*& ptr, const uint32_t version) {

            NatNet::MarkerSetModel m;
            m.name        = ReadData<std::string>::read(ptr, version);
            m.markerNames = ReadData<std::vector<std::string>>::read(ptr, version);
            return m;
        }
    };

    // Read Rigid Body Models
    template <>
    struct ReadData<NatNet::RigidBodyModel> {
        static inline NatNet::RigidBodyModel read(const char*& ptr, const uint32_t version) {

            NatNet::RigidBodyModel m;
            m.name     = version >= 0x02000000 ? ReadData<std::string>::read(ptr, version) : "";
            m.id       = ReadData<uint32_t>::read(ptr, version);
            m.parentId = ReadData<uint32_t>::read(ptr, version);
            m.offset   = ReadData<Eigen::Matrix<float, 3, 1, Eigen::DontAlign>>::read(ptr, version);
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
            auto boneModels = ReadData<std::vector<NatNet::RigidBodyModel>>::read(ptr, version);
            for (auto& model : boneModels) {
                m.boneModels[model.id] = model;
            }

            return m;
        }
    };
}  // namespace input
}  // namespace module

#endif  // MODULES_INPUT_NATNET_PARSE_H
