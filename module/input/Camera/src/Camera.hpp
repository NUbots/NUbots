/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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
#ifndef MODULE_INPUT_CAMERA_HPP
#define MODULE_INPUT_CAMERA_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <booster/idl/sensor_msgs/CameraInfo.h>
#include <booster/idl/sensor_msgs/Image.h>
#include <booster/robot/channel/channel_factory.hpp>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <nuclear>
#include <string>
#include <vector>

#include "message/input/Image.hpp"

namespace module::input {

    class Camera : public NUClear::Reactor {
    private:
        struct CameraContext {
            /// @brief DDS topic carrying sensor_msgs/Image.
            std::string image_topic{};
            /// @brief DDS topic carrying sensor_msgs/CameraInfo, always image_topic + "/camera_info".
            std::string info_topic{};
            /// @brief Name of the camera being used.
            std::string camera_name{};
            /// @brief ID of the camera being used.
            uint32_t id{0};

            /// @brief Booster SDK low-level channel for images.
            booster::robot::ChannelPtr<sensor_msgs::msg::Image> image_channel;
            /// @brief Booster SDK low-level channel for camera information.
            booster::robot::ChannelPtr<sensor_msgs::msg::CameraInfo> info_channel;

            /// @brief Lens parameters
            message::input::Image::Lens lens;
            /// @brief Mutex for lens parameters
            std::mutex lens_mutex;
            /// @brief Whether CameraInfo has ever arrived, so we can warn if the topic is absent.
            std::atomic<bool> have_camera_info{false};
            /// @brief Whether we have already warned about an out of range capture timestamp
            std::atomic<bool> warned_about_timestamp{false};
        };

        /// @brief How stale a Hcw may be before it is dropped from the buffer.
        std::atomic<std::chrono::milliseconds> hcw_buffer_duration{std::chrono::milliseconds(500)};
        /// @brief How far a capture timestamp may be from our own clock before we distrust it.
        std::atomic<std::chrono::milliseconds> max_clock_skew{std::chrono::milliseconds(1000)};

        /// @brief A list of camera contexts
        std::vector<std::unique_ptr<CameraContext>> cameras;
        /// @brief Whether the DDS channels have been created
        bool channels_created = false;

        /// @brief A mutex for sensor info.
        std::mutex sensors_mutex;
        /// @brief Recent world to camera transforms
        std::deque<std::pair<NUClear::clock::time_point, Eigen::Isometry3d>> Hcws;

        /// @brief Handle a sensor_msgs/Image sample, converting and emitting it as an Image.
        /// @param ctx The context of the camera the sample came from
        /// @param msg Pointer to the received sensor_msgs::msg::Image
        void image_handler(CameraContext& ctx, const void* msg);
        /// @brief Handle a sensor_msgs/CameraInfo sample.
        /// @param ctx The context of the camera the sample came from, its lens is updated
        /// @param msg Pointer to the received sensor_msgs::msg::CameraInfo
        void camera_info_handler(CameraContext& ctx, const void* msg);
        /// @brief The capture time from the message header, or our receive time if the clocks disagree.
        /// @param ctx The context of the camera the image came from
        /// @param image The image whose header stamp is used
        /// @return The capture time, or the current time if the stamp is unset or too far from our clock
        NUClear::clock::time_point capture_time(CameraContext& ctx, const sensor_msgs::msg::Image& image);
        /// @brief Find the buffered Hcw closest in time to the given timestamp.
        /// @param timestamp The time to find a transform for
        /// @return The nearest buffered Hcw, or identity if none are buffered
        Eigen::Isometry3d nearest_Hcw(const NUClear::clock::time_point& timestamp);

    public:
        /// @brief Called by the powerplant to build and setup the Camera reactor.
        explicit Camera(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::input

#endif  // MODULE_INPUT_CAMERA_HPP
