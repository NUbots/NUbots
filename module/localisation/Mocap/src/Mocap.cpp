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
#include "Mocap.hpp"

#include "extension/Configuration.hpp"

#include "message/input/MotionCapture.hpp"
#include "message/localisation/Field.hpp"

#include "utility/math/euler.hpp"

namespace module::localisation {

    using extension::Configuration;

    using message::input::MotionCapture;
    using message::localisation::RobotPoseGroundTruth;

    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::math::euler::rpy_intrinsic_to_mat;

    Mocap::Mocap(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Mocap.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Mocap.yaml
            this->log_level         = config["log_level"].as<NUClear::LogLevel>();
            cfg.robot_rigid_body_id = config["robot_rigid_body_id"].as<uint32_t>();
        });

        on<Trigger<MotionCapture>>().then([this](const MotionCapture& motion_capture) {
            log<DEBUG>("Motion Capture Data Received - Number of Rigid Bodies: ", motion_capture.rigid_bodies.size());
            for (const auto& rigid_body : motion_capture.rigid_bodies) {
                if (rigid_body.id == cfg.robot_rigid_body_id) {
                    log<DEBUG>("Found robot rigid body");
                    log<DEBUG>("Rigid Body: ", rigid_body.name);
                    log<DEBUG>("Rigid Body ID: ", rigid_body.id);
                    log<DEBUG>("Rigid Body Position: ", rigid_body.position);
                    log<DEBUG>("Rigid Body Rotation: ", rigid_body.rotation);
                    auto robot_pose_ground_truth = std::make_unique<message::localisation::RobotPoseGroundTruth>();

                    // Build transform from field {b} frame to torso {t} frame
                    Eigen::Isometry3d Hft = Eigen::Isometry3d::Identity();
                    Hft.translation() =
                        Eigen::Vector3d(-rigid_body.position.y(), rigid_body.position.x(), rigid_body.position.z());
                    auto rpy                      = mat_to_rpy_intrinsic(Eigen::Quaterniond(rigid_body.rotation(0),
                                                                       rigid_body.rotation(1),
                                                                       rigid_body.rotation(2),
                                                                       rigid_body.rotation(3))
                                                        .toRotationMatrix());
                    Eigen::Vector3d rpy_corrected = Eigen::Vector3d(rpy.y(), -(rpy.z() - M_PI), rpy.x());
                    Hft.linear()                  = rpy_intrinsic_to_mat(rpy_corrected);

                    robot_pose_ground_truth->Hft = Hft;
                    emit(robot_pose_ground_truth);
                }
            }
        });
    }

}  // namespace module::localisation
