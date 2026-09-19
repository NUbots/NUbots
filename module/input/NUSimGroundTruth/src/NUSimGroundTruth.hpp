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

#include <booster/idl/nav_msgs/Odometry.h>
#include <booster/robot/channel/channel_factory.hpp>
#include <nuclear>
#include <string>

namespace module::input {

    /// Bridges NUSim's test surface into NUClear: ground truth for the ball and the robot torso in
    /// (rt/nusim/gt/ball, rt/nusim/gt/robot -> message::booster::NUSimBallGroundTruth /
    /// NUSimRobotGroundTruth), and ball commands out (message::booster::NUSimBallCommand ->
    /// rt/nusim/ball_command). Only useful against NUSim; on a real robot the topics never appear.
    class NUSimGroundTruth : public NUClear::Reactor {
    public:
        explicit NUSimGroundTruth(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct Config {
            std::string ball_topic;
            std::string robot_topic;
            std::string command_topic;
        } cfg;

        void ball_handler(const void* msg);
        void robot_handler(const void* msg);

        booster::robot::ChannelPtr<nav_msgs::msg::Odometry> ball_channel;
        booster::robot::ChannelPtr<nav_msgs::msg::Odometry> robot_channel;
        booster::robot::ChannelPtr<nav_msgs::msg::Odometry> command_channel;
    };

}  // namespace module::input

#endif  // MODULE_INPUT_NUSIMGROUNDTRUTH_HPP
