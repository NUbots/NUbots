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
#ifndef MODULE_INPUT_K1CAMERA_HPP
#define MODULE_INPUT_K1CAMERA_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <booster/idl/sensor_msgs/CameraInfo.h>
#include <booster/idl/sensor_msgs/Image.h>
#include <booster/robot/channel/channel_factory.hpp>
#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <nuclear>
#include <string>
#include <vector>

#include "message/input/Image.hpp"

namespace module::input {

    class K1Camera : public NUClear::Reactor {
    private:
        struct CameraContext {
            /// @brief DDS topic carrying sensor_msgs/Image.
            std::string image_topic;
            /// @brief DDS topic carrying sensor_msgs/CameraInfo, always image_topic + "/camera_info".
            std::string info_topic;
            std::string camera_name;
            uint32_t id{0};

            booster::robot::ChannelPtr<sensor_msgs::msg::Image> image_channel;
            booster::robot::ChannelPtr<sensor_msgs::msg::CameraInfo> info_channel;

            /// @brief Lens parameters
            message::input::Image::Lens lens;
            std::mutex lens_mutex;
            /// @brief Whether CameraInfo has ever arrived, so we can warn if the topic is absent.
            std::atomic<bool> have_camera_info{false};
            /// @brief Whether we have already warned about an out of range capture timestamp
            std::atomic<bool> warned_about_timestamp{false};
        };

        std::vector<std::unique_ptr<CameraContext>> cameras;
        /// @brief Whether the DDS channels have been created
        bool channels_created = false;

        std::mutex sensors_mutex;
        /// @brief Recent world to camera transforms
        std::deque<std::pair<NUClear::clock::time_point, Eigen::Isometry3d>> Hcws;

        /// @brief Handle a sensor_msgs/Image sample, converting and emitting it as an Image.
        void image_handler(CameraContext& ctx, const void* msg);
        /// @brief Handle a sensor_msgs/CameraInfo sample.
        void camera_info_handler(CameraContext& ctx, const void* msg);
        /// @brief The capture time from the message header, or our receive time if the clocks disagree.
        NUClear::clock::time_point capture_time(CameraContext& ctx, const sensor_msgs::msg::Image& image);
        /// @brief Find the buffered Hcw closest in time to the given timestamp.
        Eigen::Isometry3d nearest_Hcw(const NUClear::clock::time_point& timestamp);

    public:
        /// @brief Called by the powerplant to build and setup the K1Camera reactor.
        explicit K1Camera(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::input

#endif  // MODULE_INPUT_K1CAMERA_HPP
