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
#include "NUSimGroundTruth.hpp"

#include <Eigen/Geometry>
#include <chrono>

#include "extension/Configuration.hpp"

#include "message/booster/NUSimGroundTruth.hpp"

#include "utility/platform/Booster/channel_factory.hpp"

namespace module::input {

    using extension::Configuration;

    using message::booster::NUSimBallCommand;
    using message::booster::NUSimBallGroundTruth;
    using message::booster::NUSimRobotGroundTruth;

    using booster::robot::ChannelFactory;
    using utility::platform::Booster::ensure_channel_factory;

    namespace {

        /// NUSim stamps ground truth with the wall clock of the physics snapshot. NUClear's clock is
        /// built on the system clock, so the stamp maps straight onto its time_point.
        NUClear::clock::time_point stamp_of(const nav_msgs::msg::Odometry& odom) {
            const auto& t = odom.header().stamp();
            return NUClear::clock::time_point(std::chrono::duration_cast<NUClear::clock::duration>(
                std::chrono::seconds(t.sec()) + std::chrono::nanoseconds(t.nanosec())));
        }

        Eigen::Vector3d vec(const geometry_msgs::msg::Vector3& v) {
            return {v.x(), v.y(), v.z()};
        }

        void set(geometry_msgs::msg::Vector3& out, const Eigen::Vector3d& v) {
            out.x(v.x());
            out.y(v.y());
            out.z(v.z());
        }

    }  // namespace

    NUSimGroundTruth::NUSimGroundTruth(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("NUSimGroundTruth.yaml").then([this](const Configuration& config) {
            log_level         = config["log_level"].as<NUClear::LogLevel>();
            cfg.ball_topic    = config["topics"]["ball"].as<std::string>();
            cfg.robot_topic   = config["topics"]["robot"].as<std::string>();
            cfg.command_topic = config["topics"]["ball_command"].as<std::string>();
        });

        on<Startup>().then("Subscribe to NUSim ground truth", [this] {
            ensure_channel_factory();
            // Ground truth is sampled state: drop stale samples rather than queue them
            ball_channel = ChannelFactory::Instance()->CreateRecvChannel<nav_msgs::msg::Odometry>(
                cfg.ball_topic,
                [this](const void* msg) { ball_handler(msg); },
                /* reliable = */ false);
            robot_channel = ChannelFactory::Instance()->CreateRecvChannel<nav_msgs::msg::Odometry>(
                cfg.robot_topic,
                [this](const void* msg) { robot_handler(msg); },
                /* reliable = */ false);
            command_channel = ChannelFactory::Instance()->CreateSendChannel<nav_msgs::msg::Odometry>(cfg.command_topic);
            log<INFO>("Listening for NUSim ground truth on", cfg.ball_topic, "and", cfg.robot_topic);
        });

        on<Trigger<NUSimBallCommand>>().then([this](const NUSimBallCommand& cmd) {
            if (command_channel == nullptr) {
                log<WARN>("Ball command before the NUSim channels were created, dropped");
                return;
            }
            nav_msgs::msg::Odometry odom;
            odom.header().frame_id(cmd.frame == NUSimBallCommand::Frame::ROBOT ? "robot" : "world");
            odom.child_frame_id(cmd.rolling ? "rolling" : "");
            odom.pose().pose().position().x(cmd.position.x());
            odom.pose().pose().position().y(cmd.position.y());
            odom.pose().pose().position().z(cmd.position.z());
            odom.pose().pose().orientation().w(1.0);
            set(odom.twist().twist().linear(), cmd.velocity);
            set(odom.twist().twist().angular(), cmd.angular_velocity);
            if (!command_channel->Write(&odom)) {
                log<WARN>("Failed to write the ball command to", cfg.command_topic);
            }
        });
    }

    void NUSimGroundTruth::ball_handler(const void* msg) {
        const auto& odom = *static_cast<const nav_msgs::msg::Odometry*>(msg);
        const auto& p    = odom.pose().pose().position();

        auto gt       = std::make_unique<NUSimBallGroundTruth>();
        gt->timestamp = stamp_of(odom);
        gt->rBSs      = Eigen::Vector3d(p.x(), p.y(), p.z());
        gt->vBs       = vec(odom.twist().twist().linear());
        gt->omegaBs   = vec(odom.twist().twist().angular());
        emit(gt);
    }

    void NUSimGroundTruth::robot_handler(const void* msg) {
        const auto& odom = *static_cast<const nav_msgs::msg::Odometry*>(msg);
        const auto& p    = odom.pose().pose().position();
        const auto& q    = odom.pose().pose().orientation();

        Eigen::Isometry3d Hst = Eigen::Isometry3d::Identity();
        Hst.linear()          = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()).normalized().toRotationMatrix();
        Hst.translation()     = Eigen::Vector3d(p.x(), p.y(), p.z());

        auto gt       = std::make_unique<NUSimRobotGroundTruth>();
        gt->timestamp = stamp_of(odom);
        gt->Hst       = Hst;
        gt->vTs       = vec(odom.twist().twist().linear());
        gt->omegaTs   = vec(odom.twist().twist().angular());
        emit(gt);
    }

}  // namespace module::input
