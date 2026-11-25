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
#ifndef MODULE_INPUT_K1SENSORS_HPP
#define MODULE_INPUT_K1SENSORS_HPP

#include <Eigen/Geometry>
#include <array>
#include <atomic>
#include <booster/idl/geometry_msgs/Pose.h>
#include <booster/robot/channel/channel_factory.hpp>
#include <memory>
#include <mutex>
#include <nuclear>
#include <string>

#include "sensors/k1_model.hpp"

#include "message/input/Sensors.hpp"
#include "message/platform/RawSensors.hpp"

namespace module::input {

    using SensorsMsg = message::input::Sensors;
    using message::platform::RawSensors;

    class Sensors : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            std::string pose_topic;
            Eigen::Isometry3d Hhp = Eigen::Isometry3d::Identity();
            Eigen::Isometry3d Hpc = Eigen::Isometry3d::Identity();
            /// @brief Deadband below which normalized odometry components are snapped to zero, to
            /// avoid tiny floating-point residuals (e.g. ~1e-10) being treated as real motion
            double odometry_deadband = 0.0;
            /// @brief Oldest odometry twist used for vTw; older than this, vTw is left unmeasured
            NUClear::clock::duration odometry_twist_max_age{};
        } cfg;

        /// @brief DDS reader for the head pose topic, created once at startup
        booster::robot::ChannelPtr<geometry_msgs::msg::Pose> pose_channel;
        /// @brief True once the reader exists, so a config reload can't create a second one
        bool channel_created = false;

        /// @brief Guards Hrh and have_pose, which are written from the DDS callback thread
        std::mutex pose_mutex;
        /// @brief Head frame in the robot base frame, from the most recent head pose message
        Eigen::Isometry3d Hrh = Eigen::Isometry3d::Identity();
        /// @brief False until the first head pose message arrives, while Hrh is still identity
        bool have_pose = false;
        /// @brief Set once the missing head pose warning has been logged
        std::atomic<bool> pose_warned{false};
        /// @brief Set once the missing odometry twist warning has been logged
        std::atomic<bool> twist_warned{false};

        std::mutex odometry_mutex;
        bool booster_odometry_has_offset = false;
        std::array<double, 3> booster_odometry_offset{};
        /// @brief Last Booster motion mode seen, used to clear the odometry offset whenever the mode
        /// changes (-1 = no mode seen yet)
        int last_booster_mode = -1;

        bool left_down   = false;
        bool middle_down = false;

        /// @brief Number of actuatable joints in the K1 robot
        static constexpr int n_servos = 22;

        /// @brief DDS callback for the head pose topic, caches the pose as Hrh
        /// @param msg The received geometry_msgs::msg::Pose
        void pose_handler(const void* msg);

        /// @brief Updates the sensors message with raw sensor data, including servo joint information
        /// @param sensors The sensors message to update
        /// @param raw_sensors The raw sensor data
        void update_raw_sensors(std::unique_ptr<SensorsMsg>& sensors, const RawSensors& raw_sensors);

    public:
        /// @brief Called by the powerplant to build and setup the Sensors reactor.
        explicit Sensors(std::unique_ptr<NUClear::Environment> environment);
        ~Sensors();
    };

}  // namespace module::input

#endif  // MODULE_INPUT_K1SENSORS_HPP
