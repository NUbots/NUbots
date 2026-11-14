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
#ifndef MODULE_INPUT_NUSIMGROUNDTRUTH_HPP
#define MODULE_INPUT_NUSIMGROUNDTRUTH_HPP

#include <Eigen/Geometry>
#include <booster/idl/nav_msgs/Odometry.h>
#include <booster/robot/channel/channel_factory.hpp>
#include <mutex>
#include <nuclear>
#include <string>

namespace module::input {

    /// @brief Reads NUSim's ground truth (rt/nusim/gt/robot and rt/nusim/gt/ball). While ground_truth is on in the
    /// config it emits Field and Ball itself and tells the vision-based localisation modules to stand down, so
    /// behaviour can be tested without vision or localisation in the loop.
    class NUSimGroundTruth : public NUClear::Reactor {
    private:
        struct Config {
            bool ground_truth = false;
            std::string robot_topic;
            std::string ball_topic;
            double world_to_field_yaw = 0.0;
            double timeout            = 0.5;
        } cfg;

        /// @brief Latest samples, written by the DDS reader threads
        std::mutex mutex;
        Eigen::Isometry3d Hst = Eigen::Isometry3d::Identity();  // torso {t} in NUSim's world {s}
        Eigen::Vector3d rBSs  = Eigen::Vector3d::Zero();
        Eigen::Vector3d vBs   = Eigen::Vector3d::Zero();
        NUClear::clock::time_point robot_time{};
        NUClear::clock::time_point ball_time{};

        booster::robot::ChannelPtr<nav_msgs::msg::Odometry> robot_channel;
        booster::robot::ChannelPtr<nav_msgs::msg::Odometry> ball_channel;

        /// @brief Whether ground truth is currently replacing vision + localisation
        bool active = false;

        void set_active(bool now_active);

    public:
        /// @brief Called by the powerplant to build and setup the NUSimGroundTruth reactor.
        explicit NUSimGroundTruth(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::input

#endif  // MODULE_INPUT_NUSIMGROUNDTRUTH_HPP
