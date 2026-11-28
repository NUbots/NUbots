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
#include "Sensors.hpp"

#include <chrono>
#include <cmath>
#include <limits>

#include "extension/Configuration.hpp"

#include "message/booster/BoosterOdometry.hpp"
#include "message/booster/BoosterOdometryTwist.hpp"
#include "message/input/Buttons.hpp"
#include "message/input/Sensors.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/platform/Booster/channel_factory.hpp"
#include "utility/platform/RawSensors.hpp"


namespace module::input {

    using booster::robot::ChannelFactory;
    using extension::Configuration;
    using message::booster::BoosterOdometry;
    using message::booster::BoosterOdometryTwist;
    using message::input::ButtonLeftDown;
    using message::input::ButtonLeftUp;
    using message::input::ButtonMiddleDown;
    using message::input::ButtonMiddleUp;
    using message::platform::RawSensors;
    using utility::input::ServoID;
    using utility::nusight::graph;
    using utility::platform::get_raw_servo;

    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::math::euler::rpy_intrinsic_to_mat;


    void Sensors::pose_handler(const void* msg) {
        const auto& pose = *static_cast<const geometry_msgs::msg::Pose*>(msg);
        const auto& p    = pose.position();
        const auto& q    = pose.orientation();

        log<DEBUG>("Head pose position xyz=", p.x(), p.y(), p.z(), "orientation xyzw=", q.x(), q.y(), q.z(), q.w());

        Eigen::Isometry3d H = Eigen::Isometry3d::Identity();
        H.translation() << p.x(), p.y(), p.z();
        H.linear() = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()).normalized().toRotationMatrix();

        std::lock_guard<std::mutex> lock(pose_mutex);
        Hrh       = H;
        have_pose = true;
    }

    Sensors::Sensors(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Sensors.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Sensors.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            // The DDS reader is created once at startup, so a reload cannot re-point it at a new
            // topic. Keep using the topic we subscribed to rather than silently ignoring the change.
            const auto pose_topic = config["head_pose"]["topic"].as<std::string>();
            if (channel_created && pose_topic != cfg.pose_topic) {
                log<WARN>("head_pose topic changed but the reader already exists, restart to apply");
            }
            else {
                cfg.pose_topic = pose_topic;
            }

            cfg.odometry_deadband      = config["odometry_deadband"].as<double>();
            cfg.odometry_twist_max_age = std::chrono::duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["odometry_twist_max_age"].as<double>()));
            cfg.odometry_twist_max_speed = config["odometry_twist_max_speed"].as<double>();
            cfg.stuck_twist_speed        = config["stuck_twist"]["speed"].as<double>();
            cfg.stationary_joint_range   = config["stuck_twist"]["stationary_joint_range"].as<double>();
            cfg.stationary_window        = std::chrono::duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["stuck_twist"]["stationary_window"].as<double>()));

            // Hpc: pitch frame to camera optical frame
            const auto& Hpc_config = config["Hpc"];
            cfg.Hpc                = Eigen::Isometry3d::Identity();
            const auto Hpc_trans   = Hpc_config["translation"];
            cfg.Hpc.translation() << Hpc_trans[0].as<double>(), Hpc_trans[1].as<double>(), Hpc_trans[2].as<double>();
            const auto Hpc_rpy = Hpc_config["rotation_rpy"];
            cfg.Hpc.linear()   = rpy_intrinsic_to_mat(
                Eigen::Vector3d(Hpc_rpy[0].as<double>(), Hpc_rpy[1].as<double>(), Hpc_rpy[2].as<double>()));

            // Hhp: head frame to pitch frame
            const auto& Hhp_config = config["Hhp"];
            cfg.Hhp                = Eigen::Isometry3d::Identity();
            const auto Hhp_trans   = Hhp_config["translation"];
            cfg.Hhp.translation() << Hhp_trans[0].as<double>(), Hhp_trans[1].as<double>(), Hhp_trans[2].as<double>();
            const auto Hhp_rpy = Hhp_config["rotation_rpy"];
            cfg.Hhp.linear()   = rpy_intrinsic_to_mat(
                Eigen::Vector3d(Hhp_rpy[0].as<double>(), Hhp_rpy[1].as<double>(), Hhp_rpy[2].as<double>()));
        });


        on<Startup>().then("Subscribe to head pose", [this] {
            utility::platform::Booster::ensure_channel_factory();

            log<INFO>("Subscribing to head pose on", cfg.pose_topic);

            pose_channel = ChannelFactory::Instance()->CreateRecvChannel<geometry_msgs::msg::Pose>(
                cfg.pose_topic,
                [this](const void* msg) { pose_handler(msg); },
                /* reliable = */ true);

            channel_created = true;
        });


        on<Trigger<RawSensors>, With<BoosterOdometry>, Optional<With<BoosterOdometryTwist>>>().then(
            [this](const RawSensors& raw_sensors,
                   const BoosterOdometry& odo,
                   const std::shared_ptr<const BoosterOdometryTwist>& twist) {
                const Eigen::Isometry2d raw = Eigen::Translation2d(odo.x, odo.y) * Eigen::Rotation2Dd(odo.theta);
                std::array<double, 3> normalized_odometry{};
                bool stationary = false;
                {
                    std::lock_guard<std::mutex> lock(odometry_mutex);
                    // Zero on the first sample so the odometry world starts at the robot
                    if (!odometry_zeroed) {
                        odometry_offset = raw;
                        odometry_zeroed = true;
                        log<INFO>("Odometry zeroed at raw pose", odo.x, odo.y, odo.theta);
                    }
                    const Eigen::Isometry2d out = odometry_offset.inverse() * raw;
                    normalized_odometry         = {out.translation().x(),
                                                   out.translation().y(),
                                                   Eigen::Rotation2Dd(out.rotation()).angle()};

                    // Stationary if no leg joint moved more than stationary_joint_range over stationary_window
                    const auto now = raw_sensors.timestamp;
                    LegJoints q;
                    for (int i = 0; i < n_leg_joints; ++i) {
                        q[i] = get_raw_servo(ServoID(first_leg_joint + i), raw_sensors).present_position;
                    }
                    leg_history.emplace_back(now, q);
                    while (leg_history.size() > 2 && leg_history[1].first <= now - cfg.stationary_window) {
                        leg_history.pop_front();
                    }
                    if (leg_history.front().first <= now - cfg.stationary_window) {
                        LegJoints lo = q;
                        LegJoints hi = q;
                        for (const auto& sample : leg_history) {
                            lo = lo.cwiseMin(sample.second);
                            hi = hi.cwiseMax(sample.second);
                        }
                        stationary = (hi - lo).maxCoeff() < cfg.stationary_joint_range;
                    }
                }

                // Snap tiny residuals to zeroes
                for (double& v : normalized_odometry) {
                    if (std::abs(v) < cfg.odometry_deadband) {
                        v = 0.0;
                    }
                }

                log<DEBUG>("Received odometry: x=" + std::to_string(odo.x) + ", y=" + std::to_string(odo.y) + ", theta="
                           + std::to_string(odo.theta) + " normalised x=" + std::to_string(normalized_odometry[0])
                           + ", y=" + std::to_string(normalized_odometry[1])
                           + ", theta=" + std::to_string(normalized_odometry[2]));


                Eigen::Isometry3d Hwr = Eigen::Isometry3d::Identity();
                Hwr.translation() << normalized_odometry[0], normalized_odometry[1], 0.0;
                // Convert yaw to rotation matrix
                Eigen::Vector3d rpy(0.0, 0.0, normalized_odometry[2]);
                Hwr.linear() = rpy_intrinsic_to_mat(rpy);

                // Hrh: head frame in robot base frame, from the most recent head pose message.
                Eigen::Isometry3d Hrh_now;
                bool got_pose = false;
                {
                    std::lock_guard<std::mutex> lock(pose_mutex);
                    Hrh_now  = Hrh;
                    got_pose = have_pose;
                }
                if (!got_pose && !pose_warned.exchange(true)) {
                    log<WARN>("No head pose received on", cfg.pose_topic, "- using identity");
                }

                // Hrc: camera optical frame in robot base frame
                Eigen::Isometry3d Hrc = Hrh_now * cfg.Hhp * cfg.Hpc;

                Eigen::Isometry3d Hwc = Hwr * Hrc;
                log<DEBUG>("Computed head pose in world frame: position xyz=",
                           Hwc.translation().x(),
                           Hwc.translation().y(),
                           Hwc.translation().z(),
                           "orientation xyzw=",
                           Eigen::Quaterniond(Hwc.linear()).x(),
                           Eigen::Quaterniond(Hwc.linear()).y(),
                           Eigen::Quaterniond(Hwc.linear()).z(),
                           Eigen::Quaterniond(Hwc.linear()).w());


                // Populate and emit the Sensors message
                auto sensors       = std::make_unique<SensorsMsg>();
                sensors->timestamp = raw_sensors.timestamp;
                sensors->Hcw       = Hwc.inverse();
                sensors->Hrw       = Hwr.inverse();

                // Update raw sensor data including servo/joint information
                update_raw_sensors(sensors, raw_sensors);

                // Compute Htw using forward kinematics.
                // compute_Htp gives Htp: Head_2 pitch_link in Trunk (base) frame.
                // Full chain: camera_optical(c) → pitch_link(p) → Trunk(t)
                //   Htc = Htp * Hpc
                // Then: Htw = Htc * Hcw  (world → camera_optical → Trunk)
                {
                    const Eigen::Isometry3d Htp = compute_Htp(sensors);
                    const Eigen::Isometry3d Htc = Htp * cfg.Hpc;
                    sensors->Htw                = Htc * sensors->Hcw;
                }

                // vTw: the controller's odometry twist, which is in its body frame (child_frame_id, by ROS
                // convention), rotated into the world by the torso's orientation. NaN without a recent, plausible
                // twist; zero while latched (see stuck_twist).
                sensors->vTw = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
                if (twist != nullptr && raw_sensors.timestamp - twist->timestamp <= cfg.odometry_twist_max_age
                    && Eigen::Vector3d(twist->linear).norm() <= cfg.odometry_twist_max_speed) {
                    Eigen::Vector3d vtw =
                        Eigen::Isometry3d(sensors->Htw).inverse().linear() * Eigen::Vector3d(twist->linear);
                    // A latched twist claims motion the legs do not show
                    if (stationary && vtw.norm() > cfg.stuck_twist_speed) {
                        vtw.setZero();
                    }
                    sensors->vTw = vtw;

                    if (log_level <= DEBUG) {
                        // Emit to plotjuggler
                        emit(graph("Sensors: vTw", vtw));
                    }
                }
                else if (twist == nullptr || raw_sensors.timestamp - twist->timestamp > cfg.odometry_twist_max_age) {
                    if (!twist_warned.exchange(true)) {
                        log<WARN>("No recent odometry twist from rt/odom - Sensors.vTw is not measured");
                    }
                }

                bool new_left_down   = raw_sensors.buttons.left;
                bool new_middle_down = raw_sensors.buttons.middle;

                if (left_down != new_left_down) {
                    left_down = new_left_down;
                    if (left_down) {
                        log<INFO>("Left Button Down");
                        emit<Scope::INLINE>(std::make_unique<ButtonLeftDown>());
                    }
                    else {
                        log<INFO>("Left Button Up");
                        emit<Scope::INLINE>(std::make_unique<ButtonLeftUp>());
                    }
                }

                if (middle_down != new_middle_down) {
                    middle_down = new_middle_down;
                    if (middle_down) {
                        log<INFO>("Middle Button Down");
                        emit<Scope::INLINE>(std::make_unique<ButtonMiddleDown>());
                    }
                    else {
                        log<INFO>("Middle Button Up");
                        emit<Scope::INLINE>(std::make_unique<ButtonMiddleUp>());
                    }
                }

                emit(sensors);
            });

        on<Shutdown>().then([this] {
            if (pose_channel != nullptr) {
                log<INFO>("Closing head pose channel");
                ChannelFactory::Instance()->CloseReader(cfg.pose_topic);
            }
        });
    }

    Sensors::~Sensors() = default;

}  // namespace module::input
