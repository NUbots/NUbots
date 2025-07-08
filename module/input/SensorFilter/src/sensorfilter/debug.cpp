/*
 * MIT License
 *
 * Copyright (c) 2025 NUbots
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

#include <fmt/format.h>

#include "SensorFilter.hpp"

#include "message/actuation/BodySide.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/input/FrameID.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"

namespace module::input {

    using message::actuation::BodySide;
    using message::input::Sensors;
    using message::localisation::RobotPoseGroundTruth;
    using message::platform::RawSensors;
    using utility::input::FrameID;

    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::nusight::graph;

    void SensorFilter::debug_sensor_filter(std::unique_ptr<Sensors>& sensors,
                                           const std::shared_ptr<const RobotPoseGroundTruth>& robot_pose_ground_truth) {
        // Raw accelerometer and gyroscope information
        emit(graph("Gyroscope", sensors->gyroscope.x(), sensors->gyroscope.y(), sensors->gyroscope.z()));
        emit(
            graph("Accelerometer", sensors->accelerometer.x(), sensors->accelerometer.y(), sensors->accelerometer.z()));

        // Foot down sensors state for each foot
        emit(graph(fmt::format("Foot Down/{}/Left", std::string(cfg.foot_down.method)),
                   sensors->feet[BodySide::LEFT].down));
        emit(graph(fmt::format("Foot Down/{}/Right", std::string(cfg.foot_down.method)),
                   sensors->feet[BodySide::RIGHT].down));
        emit(graph("Foot down phase", int(sensors->planted_foot_phase)));
        emit(graph("Anchor foot", int(planted_anchor_foot)));

        // Odometry estimates
        Eigen::Isometry3d Hwt    = Eigen::Isometry3d(sensors->Htw).inverse();
        Eigen::Vector3d est_rTWw = Hwt.translation();
        Eigen::Vector3d est_Rwt  = mat_to_rpy_intrinsic(Hwt.rotation());
        Eigen::Isometry3d Hwr    = Eigen::Isometry3d(sensors->Hrw).inverse();
        Eigen::Vector3d est_rTRw = Hwr.translation();
        Eigen::Vector3d est_Rrw  = mat_to_rpy_intrinsic(Hwr.rotation());
        Eigen::Vector3d vTw      = sensors->vTw;

        emit(graph("rTWw (estimate)", est_rTWw.x(), est_rTWw.y(), est_rTWw.z()));
        emit(graph("Rwt rpy (estimate)", est_Rwt.x(), est_Rwt.y(), est_Rwt.z()));
        emit(graph("vTw (estimate)", sensors->vTw.x(), sensors->vTw.y(), sensors->vTw.z()));
        emit(graph("rTRw (estimate)", est_rTRw.x(), est_rTRw.y(), est_rTRw.z()));
        emit(graph("Rrw rpy (estimate)", est_Rrw.x(), est_Rrw.y(), est_Rrw.z()));
        emit(graph("vTw (estimate)", vTw.x(), vTw.y(), vTw.z()));

        // Yaw filter debug information
        emit(graph("Yaw Filter/Fused Yaw", yaw_filter.get_yaw()));
        emit(graph("Yaw Filter/Bias", yaw_filter.get_bias()));

        // If we have ground truth odometry, then we can debug the error between our estimate and the ground truth
        if (robot_pose_ground_truth) {
            Eigen::Isometry3d true_Hwt = Eigen::Isometry3d(ground_truth_Hfw.inverse() * robot_pose_ground_truth->Hft);

            // Determine translation and orientation error
            Eigen::Vector3d true_rTWw  = true_Hwt.translation();
            Eigen::Vector3d error_rTWw = (true_rTWw - est_rTWw).cwiseAbs();
            Eigen::Vector3d true_Rwt   = mat_to_rpy_intrinsic(true_Hwt.rotation());
            Eigen::Vector3d error_Rwt  = (true_Rwt - est_Rwt).cwiseAbs();
            double quat_rot_error      = Eigen::Quaterniond(true_Hwt.linear() * Hwt.inverse().linear()).w();
            Eigen::Vector3d true_vTw   = Eigen::Vector3d(robot_pose_ground_truth->vTf);
            Eigen::Vector3d error_vTw  = (true_vTw - sensors->vTw).cwiseAbs();

            // Graph translation, angles and error
            emit(graph("rTWw (ground truth)", true_rTWw.x(), true_rTWw.y(), true_rTWw.z()));
            emit(graph("Rwt rpy (ground truth)", true_Rwt.x(), true_Rwt.y(), true_Rwt.z()));
            emit(graph("vTw (ground truth)", true_vTw.x(), true_vTw.y(), true_vTw.z()));
            emit(graph("rTWw translation error", error_rTWw.x(), error_rTWw.y(), error_rTWw.z()));
            emit(graph("Rwt rpy error", error_Rwt.x(), error_Rwt.y(), error_Rwt.z()));
            emit(graph("Quaternion rotational error", quat_rot_error));
            emit(graph("vTw error", error_vTw.x(), error_vTw.y(), error_vTw.z()));
        }
    }
}  // namespace module::input
