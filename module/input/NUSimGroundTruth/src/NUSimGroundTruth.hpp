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
            std::string ball_command_topic;
            double world_to_field_yaw = 0.0;
            double timeout            = 0.5;
            double warning_period     = 5.0;
        } cfg;

        /// @brief Tracks whether one of the ground truth topics is arriving, so dropouts are reported once when they
        /// happen and then every warning_period, rather than on every cycle
        struct Feed {
            /// @brief What this feed carries, for the log messages
            std::string name;
            /// @brief Whether the feed was arriving the last time it was checked
            bool fresh = false;
            /// @brief Whether a warning has been logged since the feed was last arriving
            bool warned = false;
            /// @brief When the last warning was logged. Starts at construction so a feed that never arrives gets one
            /// warning_period to show up before it is reported
            NUClear::clock::time_point last_warning = NUClear::clock::now();
        };
        Feed robot_feed{"robot pose"};
        Feed ball_feed{"ball"};

        /// @brief Latest samples, written by the DDS reader threads
        std::mutex mutex;
        Eigen::Isometry3d Hst = Eigen::Isometry3d::Identity();  // torso {t} in NUSim's world {s}
        Eigen::Vector3d rBSs  = Eigen::Vector3d::Zero();
        Eigen::Vector3d vBs   = Eigen::Vector3d::Zero();
        NUClear::clock::time_point robot_time{};
        NUClear::clock::time_point ball_time{};

        booster::robot::ChannelPtr<nav_msgs::msg::Odometry> robot_channel;
        booster::robot::ChannelPtr<nav_msgs::msg::Odometry> ball_channel;
        booster::robot::ChannelPtr<nav_msgs::msg::Odometry> ball_command_channel;

        /// @brief Whether ground truth is currently replacing vision + localisation
        bool active = false;

        void set_active(bool now_active);

        /// @brief Logs a warning when a ground truth feed drops out or never arrives, and when it comes back
        /// @param feed The feed's state
        /// @param topic The DDS topic the feed is read from
        /// @param last_received When the last sample arrived, or the epoch if none ever has
        /// @param fresh Whether the last sample is within the timeout
        /// @param consequence What the robot is doing without this feed, for the log message
        void check_feed(Feed& feed,
                        const std::string& topic,
                        const NUClear::clock::time_point& last_received,
                        bool fresh,
                        const std::string& consequence);

    public:
        /// @brief Called by the powerplant to build and setup the NUSimGroundTruth reactor.
        explicit NUSimGroundTruth(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::input

#endif  // MODULE_INPUT_NUSIMGROUNDTRUTH_HPP
