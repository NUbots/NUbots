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

#include <chrono>
#include <cmath>

#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/localisation/GroundTruthMode.hpp"

#include "utility/platform/Booster/channel_factory.hpp"

namespace module::input {

    using booster::robot::ChannelFactory;
    using extension::Configuration;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::localisation::GroundTruthMode;
    using message::localisation::RobotPoseGroundTruth;

    NUSimGroundTruth::NUSimGroundTruth(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("NUSimGroundTruth.yaml").then([this](const Configuration& config) {
            log_level              = config["log_level"].as<NUClear::LogLevel>();
            cfg.ground_truth       = config["ground_truth"].as<bool>();
            cfg.world_to_field_yaw = config["world_to_field_yaw"].as<double>();
            cfg.timeout            = config["timeout"].as<double>();
            // The DDS readers are created once at startup, so the topics only apply then
            if (robot_channel == nullptr) {
                cfg.robot_topic = config["robot_topic"].as<std::string>();
                cfg.ball_topic  = config["ball_topic"].as<std::string>();
            }
        });

        on<Startup>().then("Subscribe to NUSim ground truth", [this] {
            utility::platform::Booster::ensure_channel_factory();

            robot_channel = ChannelFactory::Instance()->CreateRecvChannel<nav_msgs::msg::Odometry>(
                cfg.robot_topic,
                [this](const void* msg) {
                    const auto& odometry = *static_cast<const nav_msgs::msg::Odometry*>(msg);
                    const auto& p        = odometry.pose().pose().position();
                    const auto& q        = odometry.pose().pose().orientation();
                    std::lock_guard<std::mutex> lock(mutex);
                    Hst.translation() << p.x(), p.y(), p.z();
                    Hst.linear() = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()).normalized().toRotationMatrix();
                    robot_time   = NUClear::clock::now();
                });
            ball_channel = ChannelFactory::Instance()->CreateRecvChannel<nav_msgs::msg::Odometry>(
                cfg.ball_topic,
                [this](const void* msg) {
                    const auto& odometry = *static_cast<const nav_msgs::msg::Odometry*>(msg);
                    const auto& p        = odometry.pose().pose().position();
                    const auto& v        = odometry.twist().twist().linear();
                    std::lock_guard<std::mutex> lock(mutex);
                    rBSs << p.x(), p.y(), p.z();
                    vBs << v.x(), v.y(), v.z();
                    ball_time = NUClear::clock::now();
                });
            log<INFO>("Listening for NUSim ground truth on", cfg.robot_topic, "and", cfg.ball_topic);
        });

        on<Shutdown>().then([this] {
            if (robot_channel != nullptr) {
                ChannelFactory::Instance()->CloseReader(cfg.robot_topic);
                ChannelFactory::Instance()->CloseReader(cfg.ball_topic);
            }
        });

        on<Every<50, Per<std::chrono::seconds>>, With<Sensors>, Single>().then([this](const Sensors& sensors) {
            Eigen::Isometry3d torso;
            Eigen::Vector3d ball_position;
            Eigen::Vector3d ball_velocity;
            bool robot_fresh = false;
            bool ball_fresh  = false;
            {
                const auto now     = NUClear::clock::now();
                const auto timeout = std::chrono::duration<double>(cfg.timeout);
                std::lock_guard<std::mutex> lock(mutex);
                torso         = Hst;
                ball_position = rBSs;
                ball_velocity = vBs;
                robot_fresh   = now - robot_time < timeout;
                ball_fresh    = now - ball_time < timeout;
            }

            // Without a simulator to ask, vision + localisation carry on as normal
            set_active(cfg.ground_truth && robot_fresh);
            if (!active) {
                return;
            }

            // NUSim world {s} -> field {f}: a yaw about the centre mark
            Eigen::Isometry3d Hfs = Eigen::Isometry3d::Identity();
            Hfs.linear() = Eigen::AngleAxisd(cfg.world_to_field_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            const Eigen::Isometry3d Hft = Hfs * torso;

            auto robot_pose = std::make_unique<RobotPoseGroundTruth>();
            robot_pose->Hft = Hft;
            emit(robot_pose);

            // Field: where the odometry world {w} sits on the field. {w} and {f} share the ground plane, so keep only
            // the planar part (x, y, yaw) of Hft * Htw.
            const Eigen::Isometry3d Hfw_full = Hft * sensors.Htw;
            const double yaw                 = std::atan2(Hfw_full(1, 0), Hfw_full(0, 0));
            Eigen::Isometry3d Hfw            = Eigen::Isometry3d::Identity();
            Hfw.linear()                     = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            Hfw.translation() = Eigen::Vector3d(Hfw_full.translation().x(), Hfw_full.translation().y(), 0.0);

            auto field         = std::make_unique<Field>();
            field->Hfw         = Hfw;
            field->covariance  = Eigen::Matrix3d::Identity() * 1e-6;
            field->uncertainty = 0.0;
            field->cost        = 0.0;
            field->localised   = true;
            emit(field);

            // Ball, in the odometry world like BallLocalisation's output
            if (ball_fresh) {
                const Eigen::Isometry3d Hwf = Hfw.inverse();
                auto ball                   = std::make_unique<Ball>();
                ball->rBWw                  = Hwf * (Hfs * ball_position);
                ball->vBw                   = Hwf.linear() * (Hfs.linear() * ball_velocity);
                ball->covariance            = Eigen::Matrix4d::Identity() * 1e-6;
                ball->confidence            = 1.0;
                ball->time_of_measurement   = NUClear::clock::now();
                ball->Hcw                   = sensors.Hcw;
                ball->average_rBWw          = ball->rBWw;
                log<DEBUG>("robot in field (x, y, yaw):",
                           Hft.translation().x(),
                           Hft.translation().y(),
                           std::atan2(Hft(1, 0), Hft(0, 0)),
                           "ball in field:",
                           (Hfs * ball_position).x(),
                           (Hfs * ball_position).y(),
                           "ball in world:",
                           ball->rBWw.x(),
                           ball->rBWw.y(),
                           "torso in world:",
                           sensors.Htw.inverse().translation().x(),
                           sensors.Htw.inverse().translation().y());
                emit(ball);
            }
        });
    }

    void NUSimGroundTruth::set_active(bool now_active) {
        if (now_active == active) {
            return;
        }
        active = now_active;
        log<INFO>(active ? "Ground truth ON: Field and Ball now come from NUSim"
                         : "Ground truth OFF: back to vision and localisation");
        auto mode     = std::make_unique<GroundTruthMode>();
        mode->enabled = active;
        emit(mode);
    }

}  // namespace module::input
